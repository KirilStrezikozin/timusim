# Thin abstract interface for logging is platform-independent.
add_library(log_interface INTERFACE)
target_include_directories(log_interface
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(
  log_interface
  INTERFACE fmt-header-only # Public API uses fmt library.
  INTERFACE span # Public API uses span.
)
