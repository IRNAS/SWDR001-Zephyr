zephyr_include_directories(.)
zephyr_library()

zephyr_library_sources(lr11xx_board.c)
#target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lr11xx_board.c)


add_subdirectory(lr11xx_driver)
target_include_directories(app PRIVATE lr11xx_driver)
add_subdirectory(radio_drivers_hal)
target_include_directories(app PRIVATE radio_drivers_hal)

