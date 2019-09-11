include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_main ${SRCS})
target_link_libraries(test_main mazelib)

add_custom_target(main
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_main/test_main
  DEPENDS test_main mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
