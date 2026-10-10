#include "retained.hpp"
#define NANOFLANN_NO_THREADS
#include <nanoflann.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace evaluation_nanoflann {
namespace {
constexpr std::int64_t max_frame = 1LL << 62;
const double infinity = std::numeric_limits<double>::infinity();

void validate_rows(std::size_t n, const double* xyz, const std::int64_t* frames) {
    if (n > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()) ||
        n > std::numeric_limits<std::size_t>::max() / 3)
        throw std::invalid_argument("row count is too large");
    if (n && (!xyz || !frames)) throw std::invalid_argument("null input buffer");
    for (std::size_t i = 0; i < n; ++i) {
        if (frames[i] < 0 || frames[i] > max_frame)
            throw std::invalid_argument("frame ID outside [0, 2**62]");
        for (std::size_t d = 0; d < 3; ++d) {
            const double v = xyz[3*i+d];
            if (!std::isfinite(v) || std::abs(v) > 1e150)
                throw std::invalid_argument("invalid coordinate");
        }
    }
}

struct Cloud {
    std::vector<double> xyz;
    std::vector<std::int64_t> frames;
    std::vector<double> confidence;
    std::size_t kdtree_get_point_count() const { return frames.size(); }
    double kdtree_get_pt(std::size_t i, std::size_t d) const { return xyz[3*i+d]; }
    template<class Bbox> bool kdtree_get_bbox(Bbox&) const { return false; }
};

struct FilteredResults {
    using Item = std::pair<double, std::size_t>;
    const Cloud& cloud;
    std::size_t capacity;
    std::int64_t frame;
    std::int64_t gap;
    double threshold;
    double radius_squared;
    std::vector<Item> selected;

    FilteredResults(const Cloud& c, std::size_t k, std::int64_t f, std::int64_t g,
                    double confidence, double radius)
        : cloud(c), capacity(k), frame(f), gap(g), threshold(confidence),
          radius_squared(radius) { selected.reserve(capacity); }
    std::size_t size() const { return selected.size(); }
    bool full() const { return size() == capacity; }
    double worstDist() const {
        double bound = radius_squared;
        if (full() && capacity) bound = std::min(bound, selected.front().first);
        // nanoflann's strict leaf comparison must include ties and the radius.
        return std::nextafter(bound, infinity);
    }
    bool addPoint(double distance, std::size_t id) {
        if (!capacity || distance > radius_squared || cloud.confidence[id] < threshold)
            return true;
        const std::int64_t other = cloud.frames[id];
        const std::int64_t difference = other >= frame ? other-frame : frame-other;
        if (gap >= 0 && difference <= gap) return true;
        const Item item(distance, id);
        if (!full()) {
            selected.push_back(item);
            std::push_heap(selected.begin(), selected.end());
        } else if (item < selected.front()) {
            std::pop_heap(selected.begin(), selected.end());
            selected.back() = item;
            std::push_heap(selected.begin(), selected.end());
        }
        return true;
    }
    void sort() { std::sort(selected.begin(), selected.end()); }
};
}

struct RetainedIndex::Impl {
    using Metric = nanoflann::L2_Simple_Adaptor<double, Cloud, double, std::size_t>;
    using Tree = nanoflann::KDTreeSingleIndexAdaptor<Metric, Cloud, 3, std::size_t>;
    Cloud cloud;
    // Member order destroys the tree before its referenced cloud.
    std::unique_ptr<Tree> tree;
    Impl(std::size_t n, const double* xyz, const std::int64_t* frames,
         const double* confidence) {
        if (n) {
            cloud.xyz.assign(xyz, xyz + 3*n);
            cloud.frames.assign(frames, frames + n);
            cloud.confidence.assign(confidence, confidence + n);
        }
        tree = std::make_unique<Tree>(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;
};

RetainedIndex::RetainedIndex(std::size_t n, const double* xyz,
                           const std::int64_t* frames, const double* confidence)
    : impl_(nullptr) {
    validate_rows(n, xyz, frames);
    if (n && !confidence) throw std::invalid_argument("null confidence buffer");
    for (std::size_t i = 0; i < n; ++i)
        if (!std::isfinite(confidence[i]) || confidence[i] < 0 || confidence[i] > 1)
            throw std::invalid_argument("invalid confidence");
    impl_ = new Impl(n, xyz, frames, confidence);
}

RetainedIndex::~RetainedIndex() { close(); }
void RetainedIndex::close() { delete impl_; impl_ = nullptr; }

void RetainedIndex::query(std::size_t m, const double* xyz, const std::int64_t* frames,
                          std::size_t k, std::int64_t gap, double threshold,
                          double radius, std::int64_t* ids, double* distances,
                          std::int64_t* counts, double* centroids) const {
    if (!impl_) throw std::runtime_error("index is closed");
    validate_rows(m, xyz, frames);
    if (!k || k > 4096 || gap < -1 || !std::isfinite(threshold) ||
        threshold < 0 || threshold > 1 || std::isnan(radius) || radius < 0 ||
        (std::isfinite(radius) && radius > 1e150))
        throw std::invalid_argument("invalid query parameters");
    if (m > std::numeric_limits<std::size_t>::max()/k)
        throw std::invalid_argument("output count is too large");
    if (m && (!ids || !distances || !counts || !centroids))
        throw std::invalid_argument("null output buffer");
    const Cloud& cloud = impl_->cloud;
    const std::size_t capacity = std::min(k, cloud.frames.size());
    for (std::size_t row = 0; row < m; ++row) {
        std::fill_n(ids + row*k, k, -1LL);
        std::fill_n(distances + row*k, k, infinity);
        std::fill_n(centroids + row*3, 3, std::numeric_limits<double>::quiet_NaN());
        FilteredResults results(cloud, capacity, frames[row], gap, threshold, radius*radius);
        if (capacity) impl_->tree->findNeighbors(results, xyz + 3*row,
                                               nanoflann::SearchParameters(0));
        counts[row] = static_cast<std::int64_t>(results.size());
        double sums[3] = {0, 0, 0};
        for (std::size_t j = 0; j < results.size(); ++j) {
            const auto& item = results.selected[j];
            ids[row*k+j] = static_cast<std::int64_t>(item.second);
            distances[row*k+j] = item.first;
            for (std::size_t d = 0; d < 3; ++d) sums[d] += cloud.xyz[3*item.second+d];
        }
        if (results.size())
            for (std::size_t d = 0; d < 3; ++d)
                centroids[3*row+d] = sums[d] / static_cast<double>(results.size());
    }
}
}
