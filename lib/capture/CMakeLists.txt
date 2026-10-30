# Header-only capture stream format.
add_library(capture INTERFACE)
target_include_directories(capture
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(capture INTERFACE imu_interface)
