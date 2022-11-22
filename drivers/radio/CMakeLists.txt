zephyr_include_directories(.)
zephyr_library()

# disable warning from lib
zephyr_library_compile_definitions(LR11XX_DISABLE_WARNINGS)

zephyr_library_sources(lr11xx_board.c)
zephyr_library_sources(lr1120mb1dxs_pa_pwr_cfg/lr1120mb1dxs_pa_pwr_cfg.c)


add_subdirectory(lr11xx_driver)
target_include_directories(app PRIVATE lr11xx_driver)
add_subdirectory(radio_drivers_hal)
target_include_directories(app PRIVATE radio_drivers_hal)

