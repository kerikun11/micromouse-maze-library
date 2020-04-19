## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## search
file(GLOB SRCS *.cpp)
add_executable(example_search ${SRCS})
target_link_libraries(example_search mazelib)
add_custom_target(search
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_search
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_search mazelib
)
