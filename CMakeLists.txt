cmake_minimum_required (VERSION 3.0)
project(PaticleFilterSLAM)

set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()

include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_executable(runSLAM ${SOURCES})
target_link_libraries (runSLAM Eigen3::Eigen)
target_link_libraries(runSLAM ${OpenCV_LIBS})