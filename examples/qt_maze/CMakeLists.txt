## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.14

find_package(Qt5 COMPONENTS Widgets)
if(NOT Qt5_FOUND)
  message(WARNING "Qt5 Not Found. skipping...")
  RETURN()
endif()

# find code files
file(GLOB_RECURSE CODE_FILES *.cpp)

# define new target
set(TARGET_NAME example_qt_maze)
add_executable(${TARGET_NAME}
  ${CODE_FILES}
)
target_link_libraries(${TARGET_NAME}
  Qt5::Widgets
  ${MICROMOUSE_MAZE_LIBRARY}
)
## qt_maze
add_custom_target(qt_maze
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS ${TARGET_NAME}
  USES_TERMINAL
)
