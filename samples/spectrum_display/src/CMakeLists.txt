zephyr_include_directories(.)
add_subdirectory(curve_plot)

target_sources(app PRIVATE main_spectrum_display.c)