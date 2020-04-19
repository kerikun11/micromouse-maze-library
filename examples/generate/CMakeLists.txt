## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## generate
file(GLOB SRCS *.cpp)
add_executable(example_generate ${SRCS})
target_link_libraries(example_generate mazelib)
add_custom_target(generate
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_generate
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_generate mazelib
)
