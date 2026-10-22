#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style>;

void joint_step(const double*, const double*, double, double*);

PYBIND11_MODULE(eigen_step_native, module) {
    auto validate = [](const Array& jacobian, const Array& delta, Array& output) {
        if (jacobian.ndim() != 2 || jacobian.shape(0) != 6 ||
            jacobian.shape(1) != 7 || delta.ndim() != 1 ||
            delta.shape(0) != 6 || output.ndim() != 1 || output.shape(0) != 7) {
            throw py::value_error("Expected shapes (6, 7), (6,), and (7,)");
        }
        if (!output.writeable()) {
            throw py::value_error("Output must be writable");
        }
    };
    module.def("scalar_probe", [](double value) { return value * value; });
    module.def("array_probe", [validate](const Array& jacobian, const Array& delta,
                                         double, Array& output) {
        validate(jacobian, delta, output);
    }, py::arg("jacobian").noconvert(), py::arg("delta").noconvert(),
       py::arg("damping"), py::arg("output").noconvert());
    module.def("joint_step", [validate](const Array& jacobian, const Array& delta,
                                       double damping, Array& output) {
        validate(jacobian, delta, output);
        joint_step(jacobian.data(), delta.data(), damping, output.mutable_data());
    }, py::arg("jacobian").noconvert(), py::arg("delta").noconvert(),
       py::arg("damping"), py::arg("output").noconvert());
}
