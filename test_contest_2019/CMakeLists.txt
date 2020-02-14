include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_contest_2019 ${SRCS})
target_link_libraries(test_contest_2019 mazelib)

add_custom_target(contest
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_contest_2019/test_contest_2019
  DEPENDS test_contest_2019 mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
