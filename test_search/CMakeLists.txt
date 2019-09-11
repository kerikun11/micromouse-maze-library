include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_search ${SRCS})
target_link_libraries(test_search mazelib)

add_custom_target(search
  ${CMAKE_BINARY_DIR}/test_search/test_search
  DEPENDS test_search mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
