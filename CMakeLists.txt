cmake_minimum_required(VERSION 3.31)
project(esp32_icm20948_device_driver)

set(CMAKE_CXX_STANDARD 20)

add_executable(esp32_icm20948_device_driver esp32-icm20948.c)
