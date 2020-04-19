## author: Ryotaro Onuki <kerikun11+github@gmail.com>
include_directories(${CMAKE_HOME_DIRECTORY}/include)
## search
add_executable(search_main search_main.cpp)
target_link_libraries(search_main mazelib)
add_custom_target(search
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/search_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS search_main mazelib
)
