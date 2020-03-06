include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_load ${SRCS})
target_link_libraries(test_load mazelib)

add_custom_target(load
  ${CMAKE_BINARY_DIR}/test_load/test_load
  DEPENDS test_load mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
