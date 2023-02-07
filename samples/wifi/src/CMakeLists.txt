zephyr_include_directories(.)

add_subdirectory(wifi_scan)
add_subdirectory(wifi_scan_time_limit)
add_subdirectory(wifi_scan_country_code)
add_subdirectory(wifi_scan_country_code_time_limit)

target_sources(app PRIVATE main_wifi.c)
target_sources(app PRIVATE wifi_result_printers.c)