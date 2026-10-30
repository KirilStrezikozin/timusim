# Header-only chip models: the LSM9DS1 signal path and its filters.
add_library(chip INTERFACE)
target_include_directories(chip INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(chip INTERFACE eigen imu_interface)
