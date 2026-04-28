cmake_minimum_required(VERSION 3.20)
project(missile_sim LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(pybind11 QUIET) # not needed, but convenient for python <-> C++
find_package(Gnuplot QUIET)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(simulation 
            src/models/evader.cpp 
            src/models/AMRAAM.cpp
            src/main.cpp)

target_include_directories(simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# For trajectory plotting
find_program(FFMPEG_EXECUTABLE ffmpeg
    HINTS
        /opt/homebrew/bin        # Apple Silicon brew
        /usr/local/bin           # Intel brew
        /usr/bin                 # Linux system install
)

# add libraries to main simulation
target_link_libraries(simulation PRIVATE Eigen3::Eigen)
target_precompile_headers(simulation PRIVATE
  <Eigen/Dense>
  <Eigen/Geometry>
  <vector>
  <cmath>
)

# Plotting Capability
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)
FetchContent_Declare(
  matplotplusplus
  GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
  GIT_TAG v1.2.1   # or latest stable
)
FetchContent_MakeAvailable(matplotplusplus)

target_link_libraries(simulation PRIVATE Matplot++::matplot)