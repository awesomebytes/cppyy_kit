#include "kernel.hpp"
double sum_sq(const double* x, std::size_t n) {
    double total = 0;
    for (std::size_t i = 0; i < n; ++i)
        total += x[i] * x[i];
    return total;
}
