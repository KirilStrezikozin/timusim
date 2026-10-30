set(LIBRARY_NAME fusion_madgwick)

add_library(${LIBRARY_NAME} FusionAhrs.c FusionBias.c FusionCompass.c)

target_compile_options(${LIBRARY_NAME} PRIVATE -Wall -Wextra -Werror)

target_include_directories(${LIBRARY_NAME}
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_definitions(${LIBRARY_NAME} PRIVATE FUSION_USE_NORMAL_SQRT)

if(UNIX AND NOT APPLE)
  target_link_libraries(${LIBRARY_NAME} m) # link math library for Linux
endif()
