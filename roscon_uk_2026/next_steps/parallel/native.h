#pragma once
#include <cstdint>
#include <cstddef>
// Only these declarations enter Cling. Template libraries stay in native.cpp.
extern "C" {
void detection_aos(std::uintptr_t records, std::size_t n, std::size_t frames,
                   double radius2, double threshold, int parallel, int workers,
                   unsigned char* flags, std::uint64_t* counts, int* peak);
void detection_columns(const double* x, const double* y, const double* z,
                       const double* confidence, const std::int64_t* frame,
                       std::size_t n, std::size_t frames, double radius2,
                       double threshold, int simd, unsigned char* flags,
                       std::uint64_t* counts);
std::size_t detection_lanes();
}
