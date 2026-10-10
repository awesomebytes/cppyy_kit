#include "native.h"
#include "detection_schema.h"
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/task_arena.h>
#include <oneapi/tbb/global_control.h>
#include <xsimd/xsimd.hpp>
#include <algorithm>
#include <atomic>
#include <stdexcept>

// DetectionType is emitted from the Pydantic model by build.py.
static bool selected(double x, double y, double z, double c, double r2, double t) {
    return (c >= t) & (((x*x + y*y) + z*z) <= r2);
}

void detection_aos(std::uintptr_t records, std::size_t n, std::size_t frames,
                   double radius2, double threshold, int parallel, int workers,
                   unsigned char* flags, std::uint64_t* counts, int* peak) {
    const auto* __restrict d = reinterpret_cast<const DetectionType*>(records);
    std::fill(counts, counts + frames, 0);
    std::atomic<int> active{0}, observed{0};
    auto range = [&](std::size_t begin, std::size_t end) {
        // Instrumentation is optional; benchmarks pass nullptr.
        if (peak) {
            int now = ++active, old = observed.load();
            while (old < now && !observed.compare_exchange_weak(old, now)) {}
        }
        for (std::size_t i = begin; i < end; ++i)
            flags[i] = selected(d[i].x, d[i].y, d[i].z, d[i].confidence, radius2, threshold);
        if (peak) --active;
    };
    if (parallel) {
        if (workers < 1 || workers > 32) throw std::invalid_argument("workers outside [1,32]");
        oneapi::tbb::global_control limit(oneapi::tbb::global_control::max_allowed_parallelism, workers);
        oneapi::tbb::task_arena arena(workers);
        arena.execute([&] {
            oneapi::tbb::parallel_for(oneapi::tbb::blocked_range<std::size_t>(0, n, 4096),
                [&](const oneapi::tbb::blocked_range<std::size_t>& r) { range(r.begin(), r.end()); });
        });
    } else range(0, n);
    // The histogram is shared with the serial baseline. No concurrent increments.
    for (std::size_t i = 0; i < n; ++i) counts[d[i].frame] += flags[i];
    if (peak) *peak = observed.load();
}

void detection_columns(const double* __restrict x, const double* __restrict y, const double* __restrict z,
                       const double* __restrict confidence, const std::int64_t* __restrict frame,
                       std::size_t n, std::size_t frames, double radius2,
                       double threshold, int simd, unsigned char* __restrict flags,
                       std::uint64_t* __restrict counts) {
    std::fill(counts, counts + frames, 0);
    std::size_t i = 0;
    if (simd) {
        using B = xsimd::batch<double>;
        constexpr std::size_t lanes = B::size;
        const B r2(radius2), t(threshold), yes(1.), no(0.);
        // Unaligned input is supported. FMA contraction is disabled at build.
        alignas(64) double result[lanes];
        for (; i + lanes <= n; i += lanes) {
            auto xx = B::load_unaligned(x + i), yy = B::load_unaligned(y + i);
            auto zz = B::load_unaligned(z + i), cc = B::load_unaligned(confidence + i);
            auto mask = (cc >= t) & (((xx*xx + yy*yy) + zz*zz) <= r2);
            xsimd::select(mask, yes, no).store_unaligned(result);
            for (std::size_t j = 0; j < lanes; ++j) flags[i+j] = static_cast<unsigned char>(result[j]);
        }
    }
    for (; i < n; ++i) flags[i] = selected(x[i], y[i], z[i], confidence[i], radius2, threshold);
    for (i = 0; i < n; ++i) counts[frame[i]] += flags[i];
}

std::size_t detection_lanes() { return xsimd::batch<double>::size; }
