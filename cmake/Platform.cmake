# Include this file in the top-level CMakeLists.txt to select the target
# platform for the build.

set(ALLOWED_TARGET_PLATFORMS "ARDUINO" "HOST")
set(TARGET_PLATFORM
    "HOST"
    CACHE STRING "Target hardware: ${ALLOWED_TARGET_PLATFORMS}")
set_property(CACHE TARGET_PLATFORM PROPERTY STRINGS ${ALLOWED_TARGET_PLATFORMS})

if(TARGET_PLATFORM IN_LIST ALLOWED_TARGET_PLATFORMS)
  # WARN: this will add a compile definition for all targets in the project,
  # even third-party ones.
  add_compile_definitions("TARGET_PLATFORM_${TARGET_PLATFORM}")
  message(STATUS "Building for target platform: ${TARGET_PLATFORM}")
else()
  message(
    FATAL_ERROR
      "Invalid TARGET_PLATFORM: ${TARGET_PLATFORM}. Must be ${ALLOWED_TARGET_PLATFORMS}"
  )
endif()
