zephyr_include_directories(.)
zephyr_library()

add_subdirectory(lr11xx_driver)
target_include_directories(app PRIVATE lr11xx_driver)
add_subdirectory(radio_drivers_hal)
target_include_directories(app PRIVATE radio_drivers_hal)

zephyr_library_sources(lr11xx_board.c)