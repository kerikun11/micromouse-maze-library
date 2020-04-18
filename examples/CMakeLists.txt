## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## contest
add_executable(contest_main contest_main.cpp)
target_link_libraries(contest_main mazelib)
add_custom_target(contest
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/contest_main
  DEPENDS contest_main mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
## generate
add_executable(generate_main generate_main.cpp)
target_link_libraries(generate_main mazelib)
add_custom_target(generate
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/generate_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS generate_main mazelib
)
## load
add_executable(load_main load_main.cpp)
target_link_libraries(load_main mazelib)
add_custom_target(load
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/load_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS load_main mazelib
)
## main
add_executable(main_main main_main.cpp)
target_link_libraries(main_main mazelib)
add_custom_target(main
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/main_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS main_main mazelib
)
## meas
add_executable(meas_main meas_main.cpp)
target_link_libraries(meas_main mazelib)
add_custom_target(meas
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/meas_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS meas_main mazelib
)
add_custom_target(meas_plot python ${CMAKE_CURRENT_SOURCE_DIR}/meas_plot.py
  WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}
)
## pi
add_executable(pi_main pi_main.cpp)
target_link_libraries(pi_main mazelib)
add_custom_target(pi
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/pi_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS pi_main mazelib
)
## search
add_executable(search_main search_main.cpp)
target_link_libraries(search_main mazelib)
add_custom_target(search
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/search_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS search_main mazelib
)
