add_executable(timusim-sim main.cpp)

target_compile_options(
  timusim-sim PRIVATE -Wall -Wextra -Werror
                      $<$<CXX_COMPILER_ID:GNU>:-Wno-dangling-reference>)

target_link_libraries(timusim-sim PRIVATE capture chip hand tomlplusplus)
