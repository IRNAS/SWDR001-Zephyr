zephyr_include_directories(.)

zephyr_library()

# Disable all warnings for Semtech code.
#
# Zephyr is compiled with a lot more warnings enabled then the basics modem.
# Since we copy the Basics Modem directly with no modifications, the warnings clog up
# the build output in our applications.
zephyr_library_compile_options(-w)


# Library flag that disables some warnings
zephyr_library_compile_definitions(LR11XX_DISABLE_WARNINGS)

zephyr_library_sources(lr11xx_bootloader.c)
zephyr_library_sources(lr11xx_crypto_engine.c)
zephyr_library_sources(lr11xx_driver_version.c)
zephyr_library_sources(lr11xx_gnss.c)
zephyr_library_sources(lr11xx_lr_fhss.c)
zephyr_library_sources(lr11xx_radio.c)
zephyr_library_sources(lr11xx_radio_timings.c)
zephyr_library_sources(lr11xx_regmem.c)
zephyr_library_sources(lr11xx_system.c)
zephyr_library_sources(lr11xx_wifi.c)
