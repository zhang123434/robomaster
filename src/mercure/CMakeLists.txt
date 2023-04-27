Cmake_minimum_required (VERSION 2.8)


project(mercure_driver)

message(STATUS "Configuring Sub_project:mercure_drive")

find_package(OpenCV 4 REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

#detect_factory

#aux_source_directory(./src MERCURE_
add_library(mercure_driver mercure_driver.cpp)

target_link_libraries(mercure_driver ${OpenCV_LIBS} gxiapi)


