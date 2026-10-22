#include <cstddef>
#include <cstdint>

// Internal benchmark entry point. The shared Python adapter validates and
// retains all arrays before passing their addresses. Output must not alias input.
void cloud_transform(std::uintptr_t source, std::uintptr_t transform,
                     std::uintptr_t destination, std::size_t count) {
    const auto* points = reinterpret_cast<const double*>(source);
    const auto* t = reinterpret_cast<const double*>(transform);
    auto* out = reinterpret_cast<double*>(destination);
    for (std::size_t i = 0; i < count; ++i) {
        const double x = points[3*i], y = points[3*i+1], z = points[3*i+2];
        const double a = t[0]*x + t[1]*y + t[2]*z + t[3];
        const double b = t[4]*x + t[5]*y + t[6]*z + t[7];
        const double c = t[8]*x + t[9]*y + t[10]*z + t[11];
        out[4*i] = a;
        out[4*i+1] = b;
        out[4*i+2] = c;
        out[4*i+3] = a*a + b*b + c*c;
    }
}
