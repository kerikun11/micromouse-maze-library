include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_meas ${SRCS})
target_link_libraries(test_meas mazelib)

add_custom_target(meas
  ${CMAKE_BINARY_DIR}/test_meas/test_meas
  DEPENDS test_meas mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
