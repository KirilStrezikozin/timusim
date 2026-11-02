# Root CMakeLists.txt. Builds the host simulator and examples/quickstart; the
# capture firmware under examples/ builds with PlatformIO. See README.md.

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

set_property(GLOBAL PROPERTY C_STANDARD_REQUIRED YES)
set_property(GLOBAL PROPERTY CXX_STANDARD_REQUIRED YES)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(Platform)

project(
  timusim
  VERSION 0.0.1
  LANGUAGES C CXX)

enable_testing()

add_subdirectory(external)
add_subdirectory(lib)
add_subdirectory(sim)
add_subdirectory(examples/quickstart)
add_subdirectory(tests)
