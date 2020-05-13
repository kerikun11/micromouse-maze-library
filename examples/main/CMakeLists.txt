## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_main ${SRCS})
target_include_directories(example_main PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(example_main mazelib)
## main
add_custom_target(main
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/example_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_main mazelib
  USES_TERMINAL
)
