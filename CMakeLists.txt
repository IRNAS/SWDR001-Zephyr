cmake_minimum_required(VERSION 3.13.1)

zephyr_compile_options(-fdiagnostics-color=always)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(drivers)

# We require the .h files in the dts directory to be available to the C compiler.
zephyr_include_directories(dts)
