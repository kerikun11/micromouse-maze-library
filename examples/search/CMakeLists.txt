## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.14

## give a name
set(CUSTOM_TARGET_NAME "search")
set(TARGET_NAME example_${CUSTOM_TARGET_NAME})
## make a executable
file(GLOB SRC_FILES *.cpp)
add_executable(${TARGET_NAME} ${SRC_FILES})
target_link_libraries(${TARGET_NAME} PRIVATE ${MICROMOUSE_MAZE_LIBRARY})
## make a custom target to run example
add_custom_target(${CUSTOM_TARGET_NAME}
  COMMAND ${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
