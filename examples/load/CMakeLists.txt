## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## load
file(GLOB SRCS *.cpp)
add_executable(example_load ${SRCS})
target_link_libraries(example_load mazelib)
add_custom_target(load
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_load
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_load mazelib
)
