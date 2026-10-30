# Header-only hand kinematics: chain, move profiles, trajectories.
add_library(hand INTERFACE)
target_include_directories(hand INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(hand INTERFACE eigen)
