#include "smoother.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::invalid_argument("usage: smoother ALPHA [SAMPLE ...]");
        auto number = [](const char* text) {
            std::size_t end = 0;
            const double value = std::stod(text, &end);
            if (text[end] != '\0') throw std::invalid_argument("invalid number");
            return value;
        };
        component_example::Config config;
        config.smoothing_weight = number(argv[1]);
        component_example::Smoother smoother(config);
        std::vector<double> inputs;
        for (int i = 2; i < argc; ++i) inputs.push_back(number(argv[i]));
        std::cout << std::setprecision(17);
        for (double value : smoother.process(inputs)) std::cout << value << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
