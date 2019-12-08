include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_contest ${SRCS})
target_link_libraries(test_contest mazelib)

add_custom_target(contest
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_contest/test_contest
  DEPENDS test_contest mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
