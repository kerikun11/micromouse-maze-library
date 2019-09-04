include_directories(../include)
include_directories("${gtest_SOURCE_DIR}/include")

file(GLOB SRCS *.cpp)
add_executable(test_mazelib ${SRCS})
target_link_libraries(test_mazelib mazelib gtest)
