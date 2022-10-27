zephyr_include_directories(.)
add_subdirectory(almanac)

target_sources(app PRIVATE main.c)