## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_pi ${SRCS})
target_include_directories(example_pi PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(example_pi mazelib)
## pi
add_custom_target(pi
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/example_pi
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_pi mazelib
  USES_TERMINAL
)
