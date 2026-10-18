cmake_minimum_required(VERSION 3.16)
project(tiago_pro_harmonic_native LANGUAGES CXX)
find_package(gz-sim8 REQUIRED)
add_library(tiago_pro_physical_mimic SHARED PhysicalMimic.cc)
target_compile_features(tiago_pro_physical_mimic PRIVATE cxx_std_17)
target_link_libraries(tiago_pro_physical_mimic PRIVATE gz-sim8::gz-sim8)
# Conda's imported target includes GUI/media libraries. Keep only symbols used
# by this server plugin so loading it does not pull a second GUI/audio stack.
target_link_options(tiago_pro_physical_mimic PRIVATE "LINKER:--as-needed")
