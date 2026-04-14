# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-src")
  file(MAKE_DIRECTORY "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-src")
endif()
file(MAKE_DIRECTORY
  "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-build"
  "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix"
  "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix/tmp"
  "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix/src/matplotplusplus-populate-stamp"
  "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix/src"
  "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix/src/matplotplusplus-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix/src/matplotplusplus-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/_deps/matplotplusplus-subbuild/matplotplusplus-populate-prefix/src/matplotplusplus-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
