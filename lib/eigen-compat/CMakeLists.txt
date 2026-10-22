# Header-only Eigen compatibility layer library.
add_library(eigen-compat INTERFACE)
target_include_directories(eigen-compat
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(eigen-compat INTERFACE eigen)
