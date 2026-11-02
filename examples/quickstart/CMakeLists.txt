add_executable(timusim-quickstart main.cpp)

target_compile_options(
  timusim-quickstart PRIVATE -Wall -Wextra -Werror
                             $<$<CXX_COMPILER_ID:GNU>:-Wno-dangling-reference>)

target_include_directories(timusim-quickstart
                           PRIVATE "${CMAKE_SOURCE_DIR}/sim")

target_link_libraries(timusim-quickstart PRIVATE capture chip hand tomlplusplus)
