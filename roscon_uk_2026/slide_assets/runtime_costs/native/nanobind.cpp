#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace nb = nanobind;
using Jacobian = nb::ndarray<nb::numpy, const double, nb::shape<6, 7>,
                             nb::c_contig, nb::device::cpu>;
using Delta = nb::ndarray<nb::numpy, const double, nb::shape<6>,
                         nb::c_contig, nb::device::cpu>;
using Output = nb::ndarray<nb::numpy, double, nb::shape<7>,
                          nb::c_contig, nb::device::cpu>;

void joint_step(const double*, const double*, double, double*);

NB_MODULE(eigen_step_nanobind, module) {
    module.def("scalar_probe", [](double value) { return value * value; });
    module.def("array_probe", [](Jacobian, Delta, double, Output) {},
       nb::arg("jacobian").noconvert(), nb::arg("delta").noconvert(),
       nb::arg("damping"), nb::arg("output").noconvert());
    module.def("joint_step", [](Jacobian jacobian, Delta delta,
                                double damping, Output output) {
        joint_step(jacobian.data(), delta.data(), damping, output.data());
    }, nb::arg("jacobian").noconvert(), nb::arg("delta").noconvert(),
       nb::arg("damping"), nb::arg("output").noconvert());
}
