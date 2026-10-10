#include "native.hpp"
#define NANOFLANN_NO_THREADS
#include <nanoflann.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace recorded_nn {
using Clock = std::chrono::steady_clock;
static double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
static void valid_xyz(const double* xyz, std::size_t n) {
    for (std::size_t i = 0; i < 3*n; ++i)
        if (!std::isfinite(xyz[i]) || std::abs(xyz[i]) > 1e150)
            throw std::invalid_argument("coordinates must be finite and abs <= 1e150");
}
struct Index::Impl {
    struct Cloud {
        std::vector<double> xyz;
        std::vector<std::int64_t> frames;
        std::vector<double> confidence;
        std::size_t kdtree_get_point_count() const { return frames.size(); }
        double kdtree_get_pt(std::size_t i, std::size_t axis) const {
            return xyz[3*i + axis];
        }
        template<class Box> bool kdtree_get_bbox(Box&) const { return false; }
    } cloud;
    using Tree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, Cloud, double, std::size_t>, Cloud, 3, std::size_t>;
    std::unique_ptr<Tree> tree;
    double build_time;
    Impl(const double* xyz, const std::int64_t* frames,
         const double* confidence, std::size_t n) {
        const auto start = Clock::now();
        valid_xyz(xyz, n);
        for (std::size_t i = 0; i < n; ++i) {
            if (frames[i] < 0 || frames[i] > (std::int64_t(1) << 62))
                throw std::invalid_argument("frame must be in [0, 2**62]");
            if (!std::isfinite(confidence[i]) || confidence[i] < 0 || confidence[i] > 1)
                throw std::invalid_argument("confidence must be finite in [0, 1]");
        }
        if (n) {
            cloud.xyz.assign(xyz, xyz+3*n);
            cloud.frames.assign(frames, frames+n);
            cloud.confidence.assign(confidence, confidence+n);
        }
        tree.reset(new Tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(16)));
        build_time = seconds(start);
    }
    // Rejecting ineligible points here searches past them. Postfiltering k
    // unfiltered neighbors would miss eligible neighbors farther away.
    struct Selection {
        const Cloud& cloud;
        std::size_t capacity;
        std::int64_t frame, gap;
        double min_confidence, radius_squared;
        std::vector<std::pair<double, std::size_t>> best;
        Selection(const Cloud& c, std::size_t k, std::int64_t f,
                  std::int64_t g, double confidence, double radius)
            : cloud(c), capacity(k), frame(f), gap(g),
              min_confidence(confidence), radius_squared(radius) { best.reserve(k); }
        bool full() const { return best.size() == capacity; }
        std::size_t size() const { return best.size(); }
        void sort() {} // Insertion maintains (squared distance, original index).
        double worstDist() const {
            const double boundary = full() ? std::min(radius_squared, best.back().first)
                                           : radius_squared;
            // nanoflann's leaf check is strict '<'. Include boundary ties and
            // let addPoint choose by the original index, independent of traversal.
            return std::nextafter(boundary, std::numeric_limits<double>::infinity());
        }
        bool addPoint(double squared, std::size_t id) {
            if (squared > radius_squared || cloud.confidence[id] < min_confidence)
                return true;
            if (gap >= 0 && std::abs(cloud.frames[id] - frame) <= gap)
                return true;
            const auto candidate = std::make_pair(squared, id);
            const auto where = std::lower_bound(best.begin(), best.end(), candidate);
            if (best.size() < capacity || where != best.end()) {
                best.insert(where, candidate);
                if (best.size() > capacity) best.pop_back();
            }
            return true;
        }
    };
};
Index::Index(const double* xyz, const std::int64_t* frames,
             const double* confidence, std::size_t n)
    : impl_(new Impl(xyz, frames, confidence, n)) {}
Index::~Index() { delete impl_; }
void Index::close() { delete impl_; impl_ = nullptr; }
std::size_t Index::size() const {
    if (!impl_) throw std::runtime_error("index is closed");
    return impl_->cloud.frames.size();
}
double Index::build_seconds() const {
    if (!impl_) throw std::runtime_error("index is closed");
    return impl_->build_time;
}
unsigned int Index::library_version() const { return NANOFLANN_VERSION; }
double Index::query(const double* xyz, const std::int64_t* frames,
                    std::size_t m, std::size_t k, std::int64_t frame_gap,
                    double min_confidence, double max_distance,
                    std::int64_t* ids, double* d2,
                    std::int64_t* counts, double* centroids) const {
    if (!impl_) throw std::runtime_error("index is closed");
    if (!k || k > 4096) throw std::invalid_argument("k must be in [1, 4096]");
    if (frame_gap < -1 || frame_gap > (std::int64_t(1) << 62))
        throw std::invalid_argument("frame_gap must be in [-1, 2**62]");
    if (!std::isfinite(min_confidence) || min_confidence < 0 || min_confidence > 1)
        throw std::invalid_argument("min_confidence must be finite in [0, 1]");
    if (std::isnan(max_distance) || max_distance < 0 ||
        (std::isfinite(max_distance) && max_distance > 1e150))
        throw std::invalid_argument("max_distance must be in [0, 1e150] or +inf");
    valid_xyz(xyz, m);
    for (std::size_t j = 0; j < m; ++j)
        if (frames[j] < 0 || frames[j] > (std::int64_t(1) << 62))
            throw std::invalid_argument("frame must be in [0, 2**62]");
    const auto start = Clock::now();
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Impl::Selection result(impl_->cloud, std::min(k, size()), 0,
                           frame_gap, min_confidence, max_distance*max_distance);
    for (std::size_t j = 0; j < m; ++j) {
        std::fill(ids+j*k, ids+(j+1)*k, -1);
        std::fill(d2+j*k, d2+(j+1)*k, inf);
        result.frame = frames[j];
        result.best.clear();
        if (size()) impl_->tree->findNeighbors(result, xyz+3*j, nanoflann::SearchParameters(0));
        counts[j] = result.size();
        double sum[3] = {0, 0, 0};
        for (std::size_t i = 0; i < result.size(); ++i) {
            const auto item = result.best[i];
            ids[j*k+i] = item.second;
            d2[j*k+i] = item.first;
            for (std::size_t a = 0; a < 3; ++a)
                sum[a] += impl_->cloud.xyz[3*item.second+a];
        }
        for (std::size_t a = 0; a < 3; ++a)
            centroids[3*j+a] = result.size() ? sum[a]/result.size() : nan;
    }
    return seconds(start);
}
}
