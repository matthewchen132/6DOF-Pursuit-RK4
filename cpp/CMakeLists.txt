cmake_minimum_required(VERSION 3.20)
project(missile_sim LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11) # not needed, but convenient for python <-> C++
find_package(Gnuplot)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(simulation 
            src/simulation.cpp 
            src/models/wind_vector.cpp 
            src/models/evader.cpp 
            src/models/AMRAAM.cpp
            src/main.cpp)

target_include_directories(simulation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


# add libraries to main simulation
target_link_libraries(simulation PRIVATE Eigen3::Eigen)

# Plotting Capability
include(FetchContent)

FetchContent_Declare(
  matplotplusplus
  GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
  GIT_TAG v1.2.1   # or latest stable
)

FetchContent_MakeAvailable(matplotplusplus)

target_link_libraries(simulation PRIVATE Matplot++::matplot)