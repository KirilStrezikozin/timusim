# Thin abstract interface for bus communication that is platform-independent.
add_library(bus_interface INTERFACE)
target_include_directories(bus_interface
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(
  bus_interface
  INTERFACE tl # Public API uses tl library.
  INTERFACE log_interface # Public API uses log_interface for logging.
)
