add_executable(conformance-test conformance_test.cpp)

# fmt (via log_interface) trips GCC's -Wdangling-reference false positive.
target_compile_options(
  conformance-test PRIVATE -Wall -Wextra -Werror
                           $<$<CXX_COMPILER_ID:GNU>:-Wno-dangling-reference>)

target_link_libraries(
  conformance-test
  PRIVATE eigen
  PRIVATE imu_interface)

add_test(NAME conformance COMMAND conformance-test)

add_executable(capture-test capture_test.cpp)

target_compile_options(
  capture-test PRIVATE -Wall -Wextra -Werror
                       $<$<CXX_COMPILER_ID:GNU>:-Wno-dangling-reference>)

target_link_libraries(capture-test PRIVATE capture)

add_test(NAME capture COMMAND capture-test)

add_executable(hand-test hand_test.cpp)

target_compile_options(hand-test PRIVATE -Wall -Wextra -Werror)

target_link_libraries(hand-test PRIVATE hand)

add_test(NAME hand COMMAND hand-test)

add_executable(chip-test chip_test.cpp)

target_compile_options(
  chip-test PRIVATE -Wall -Wextra -Werror
                    $<$<CXX_COMPILER_ID:GNU>:-Wno-dangling-reference>)

target_link_libraries(chip-test PRIVATE chip)

add_test(NAME chip COMMAND chip-test)

add_executable(sim-imu-test sim_imu_test.cpp)

target_compile_options(
  sim-imu-test PRIVATE -Wall -Wextra -Werror
                       $<$<CXX_COMPILER_ID:GNU>:-Wno-dangling-reference>)

target_link_libraries(sim-imu-test PRIVATE chip hand)

add_test(NAME sim-imu COMMAND sim-imu-test)

add_executable(tremor-test tremor_test.cpp)

target_compile_options(tremor-test PRIVATE -Wall -Wextra -Werror)

target_link_libraries(tremor-test PRIVATE hand)

add_test(NAME tremor COMMAND tremor-test)
