"""Expose the reduction directly through cppyy."""
import cppyy
from measure import run

cppyy.cppdef("""
double sum_sq(const double* x, size_t n) {
    double total = 0;
    for (size_t i = 0; i < n; ++i) total += x[i] * x[i];
    return total;
}
""")


if __name__ == '__main__':
    run(lambda x: cppyy.gbl.sum_sq(x, x.size))
