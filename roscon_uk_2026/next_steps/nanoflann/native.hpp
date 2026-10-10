#include <cstddef>
#include <cstdint>
namespace recorded_nn {
class Index {
    struct Impl;
    Impl* impl_;
public:
    Index(const double* xyz, const std::int64_t* frames,
          const double* confidence, std::size_t n);
    ~Index();
    Index(const Index&) = delete;
    Index& operator=(const Index&) = delete;
    void close();
    std::size_t size() const;
    double build_seconds() const;
    unsigned int library_version() const;
    double query(const double* xyz, const std::int64_t* frames,
                 std::size_t m, std::size_t k, std::int64_t frame_gap,
                 double min_confidence, double max_distance,
                 std::int64_t* ids, double* distances_squared,
                 std::int64_t* counts, double* centroids) const;
};
}
