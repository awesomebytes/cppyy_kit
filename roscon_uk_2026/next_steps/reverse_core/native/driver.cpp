#include "pose_filter.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::invalid_argument("usage: pose_filter_driver CONFIG INPUT.csv OUTPUT.csv");
        const auto config = reverse_demo::load_config_file(argv[1]);
        std::ifstream input(argv[2]);
        if (!input) throw std::runtime_error("cannot open input CSV");
        std::string line;
        if (!std::getline(input, line) || line != "timestamp_ns,x_m,y_m,z_m")
            throw std::invalid_argument("unexpected CSV header");
        std::vector<std::int64_t> timestamps;
        std::vector<double> positions;
        while (std::getline(input, line)) {
            std::stringstream row(line);
            std::string field;
            std::size_t consumed;
            if (!std::getline(row, field, ',')) throw std::invalid_argument("missing timestamp");
            const auto timestamp = std::stoll(field, &consumed);
            if (consumed != field.size()) throw std::invalid_argument("invalid timestamp text");
            timestamps.push_back(timestamp);
            for (int a = 0; a < 3; ++a) {
                if (!std::getline(row, field, ',')) throw std::invalid_argument("missing coordinate");
                const double value = std::stod(field, &consumed);
                if (consumed != field.size()) throw std::invalid_argument("invalid coordinate text");
                positions.push_back(value);
            }
            if (std::getline(row, field, ',')) throw std::invalid_argument("extra CSV field");
        }
        reverse_demo::PoseEstimator estimator(config);
        std::vector<double> output(positions.size());
        estimator.process(timestamps.data(), positions.data(), timestamps.size(), output.data());
        std::ofstream result(argv[3]);
        if (!result) throw std::runtime_error("cannot open output CSV");
        result << "timestamp_ns,x_m,y_m,z_m\n" << std::setprecision(17);
        for (std::size_t i = 0; i < timestamps.size(); ++i)
            result << timestamps[i] << ',' << output[3*i] << ',' << output[3*i+1] << ',' << output[3*i+2] << '\n';
        if (!result) throw std::runtime_error("output CSV write failed");
        std::cout << "samples=" << timestamps.size() << " frame_id=" << config.output_frame << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
