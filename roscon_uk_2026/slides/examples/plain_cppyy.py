import cppyy
import numpy as np

cppyy.cppdef("""
double sum_sq(const double* x, size_t n) {
    double total = 0;
    for (size_t i = 0; i < n; ++i)
        total += x[i] * x[i];
    return total;
}
""")

x = np.array([1., 2., 3.])
print(cppyy.gbl.sum_sq(x, x.size))
