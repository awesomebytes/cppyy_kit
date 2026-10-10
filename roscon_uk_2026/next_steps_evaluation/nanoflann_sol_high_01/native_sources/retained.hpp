#ifndef EVALUATION_NANOFLANN_RETAINED_HPP
#define EVALUATION_NANOFLANN_RETAINED_HPP
#include <cstddef>
#include <cstdint>
namespace evaluation_nanoflann {
class RetainedIndex {
public:
    RetainedIndex(std::size_t n, const double* xyz,
                  const std::int64_t* frames, const double* confidence);
    ~RetainedIndex();
    RetainedIndex(const RetainedIndex&) = delete;
    RetainedIndex& operator=(const RetainedIndex&) = delete;
    RetainedIndex(RetainedIndex&&) = delete;
    RetainedIndex& operator=(RetainedIndex&&) = delete;
    void query(std::size_t m, const double* xyz, const std::int64_t* frames,
               std::size_t k, std::int64_t frame_gap, double min_confidence,
               double max_distance, std::int64_t* ids, double* distances,
               std::int64_t* counts, double* centroids) const;
    void close();
private:
    struct Impl;
    Impl* impl_;
};
}
#endif
