add_subdirectory(eigen)
add_subdirectory(tl)
add_subdirectory(fmt)

# Build the platform-independent LSM6DSL sensor driver library here, since it is
# included as a Git submodule.
add_library(lsm6dsl-pid STATIC lsm6dsl-pid/lsm6dsl_reg.c)
target_include_directories(lsm6dsl-pid
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lsm6dsl-pid")

# Same for LSM9DS1 library.
add_library(lsm9ds1-pid STATIC lsm9ds1-pid/lsm9ds1_reg.c)
target_include_directories(lsm9ds1-pid
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lsm9ds1-pid")

# toml++ v3.4.0, single header, MIT: scenario and chip parameter files.
add_library(tomlplusplus INTERFACE)
target_include_directories(tomlplusplus
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/tomlplusplus")
