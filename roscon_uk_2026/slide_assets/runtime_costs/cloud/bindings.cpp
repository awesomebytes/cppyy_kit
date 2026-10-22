#include <cstddef>
#include <cstdint>

#ifdef CLOUD_NANOBIND
#include <nanobind/nanobind.h>
#else
#include <pybind11/pybind11.h>
#endif

void cloud_transform(std::uintptr_t, std::uintptr_t, std::uintptr_t, std::size_t);

#ifdef CLOUD_NANOBIND
NB_MODULE(cloud_nanobind, module) {
#else
PYBIND11_MODULE(cloud_pybind11, module) {
#endif
    module.def("_transform_addresses", &cloud_transform);
}
