include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_generate ${SRCS})
target_link_libraries(test_generate mazelib)

add_custom_target(gen
  ${CMAKE_BINARY_DIR}/test_generate/test_generate
  DEPENDS test_generate mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
