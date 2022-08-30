# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# set path to prj.configuration and Kconfig
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../Kconfig)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(tx_cw)

zephyr_compile_options(-fdiagnostics-color=always)

zephyr_include_directories(.)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common src/common)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/radio src/radio)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})