##
## Google Test
##
# Download and unpack googletest at configure time
configure_file(../cmake/googletest_download.cmake
               ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . 
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
execute_process(COMMAND ${CMAKE_COMMAND} --build . 
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
# Prevent overriding the parent project's compiler/linker; settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

include_directories(../include)
include_directories("${gtest_SOURCE_DIR}/include")

file(GLOB SRCS *.cpp)
add_executable(test_mazelib ${SRCS})
target_link_libraries(test_mazelib mazelib gtest)

add_custom_target(gt
  ${CMAKE_BINARY_DIR}/test/test_mazelib
  DEPENDS test_mazelib mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
