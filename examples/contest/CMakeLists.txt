## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## contest
file(GLOB SRCS *.cpp)
add_executable(example_contest ${SRCS})
target_link_libraries(example_contest mazelib)
add_custom_target(contest
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/example_contest
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_contest mazelib
)
