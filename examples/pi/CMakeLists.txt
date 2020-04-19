## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## pi
file(GLOB SRCS *.cpp)
add_executable(example_pi ${SRCS})
target_link_libraries(example_pi mazelib)
add_custom_target(pi
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/example_pi
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_pi mazelib
)
