# Header-only library with math utilities.
add_library(math-utils INTERFACE)
target_include_directories(math-utils
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(
  math-utils
  INTERFACE tl # Public API uses tl library.
  INTERFACE log_interface # Public API uses log_interface for logging.
  INTERFACE fusion_madgwick # Vector and quaternion types.
)
