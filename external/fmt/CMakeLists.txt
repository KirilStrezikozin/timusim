add_library(fmt-header-only INTERFACE)
target_include_directories(fmt-header-only
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
