include_directories(../include)
file(GLOB SRCS *.cpp)
add_executable(test_generate ${SRCS})
target_link_libraries(test_generate mazelib)
