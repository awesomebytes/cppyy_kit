#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "kernel.hpp"
namespace py = pybind11;
PYBIND11_MODULE(_native, m) {
    m.def("sum_sq", [](py::array_t<double, py::array::c_style> x) {
        return sum_sq(x.data(), x.size());
    });
}
