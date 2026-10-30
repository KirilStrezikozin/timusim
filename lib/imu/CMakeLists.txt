# Thin abstract interface for IMU interaction that is platform-independent.
add_library(imu_interface INTERFACE)
target_include_directories(imu_interface
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(
  imu_interface
  INTERFACE bus_interface # Link dependant bus interface.
  INTERFACE tl # Public API uses tl library.
  INTERFACE log_interface # Public API uses log_interface for logging.
  INTERFACE time # Public API uses time library for timing utilities.
  INTERFACE math-utils # Public API uses math-utils for math utilities.
  INTERFACE lsm6dsl-pid # For LSM6DSL IMU implementation.
  INTERFACE lsm9ds1-pid # For LSM9DS1 IMU implementation.
)
