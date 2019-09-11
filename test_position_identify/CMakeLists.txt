include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_position_identify ${SRCS})
target_link_libraries(test_position_identify mazelib)

add_custom_target(pi
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_position_identify/test_position_identify
  DEPENDS test_position_identify mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
