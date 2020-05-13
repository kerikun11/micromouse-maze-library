## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_executable(example_meas ${SRCS})
target_include_directories(example_meas PRIVATE ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(example_meas mazelib)
## meas
add_custom_target(meas
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/example_meas
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS example_meas mazelib
  USES_TERMINAL
)
## meas_plot
add_custom_target(meas_plot python ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
