## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## main
file(GLOB SRCS *.cpp)
add_executable(example_main ${SRCS})
target_link_libraries(example_main mazelib)
add_custom_target(main
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/example_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_main mazelib
)
