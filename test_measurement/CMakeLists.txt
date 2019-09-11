include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_measurement ${SRCS})
target_link_libraries(test_measurement mazelib)

add_custom_target(meas
  ${CMAKE_BINARY_DIR}/test_measurement/test_measurement
  DEPENDS test_measurement mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
