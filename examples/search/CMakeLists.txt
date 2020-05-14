# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.05.14

# give a name
set(CUSTOM_TARGET_NAME "search")
set(TARGET_NAME example_${CUSTOM_TARGET_NAME})
# make a target
file(GLOB SRC_FILES *.cpp)
add_executable(${TARGET_NAME} ${SRC_FILES})
target_include_directories(${TARGET_NAME}
  PRIVATE ${CMAKE_HOME_DIRECTORY}/include
)
target_link_libraries(${TARGET_NAME} mazelib)
# make a custom target to run
add_custom_target(${CUSTOM_TARGET_NAME}
  COMMAND clear && ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS ${TARGET_NAME} mazelib
  USES_TERMINAL
)
