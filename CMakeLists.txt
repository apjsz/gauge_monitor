cmake_minimum_required(VERSION 3.1)
project(gauge_monitor)
set(DCMAKE_BUILD_TYPE = Release)

find_package(OpenCV REQUIRED)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(monitor ${SOURCES})

target_link_libraries(monitor PRIVATE ${OpenCV_LIBS})

