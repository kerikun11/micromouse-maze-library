##
## file: CMakeLists.txt
## author: Ryotaro Onuki
## date: 2019.05.07
##
cmake_minimum_required(VERSION 3.10)
project("MazeLib" CXX)

##
## build option for compiler
##
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wfatal-errors -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -O2")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")

## for __attribute__((__packed__)) on MSYS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-ms-bitfields")

## more warnnings
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wdisabled-optimization -Wfloat-equal -Wformat -Wformat=2 -Wformat-nonliteral -Wformat-security -Wformat-y2k -Wimport -Winit-self -Winvalid-pch -Wmissing-braces -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -pedantic-errors")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunsafe-loop-optimizations")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winline")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wlong-long")

##
## Google Test
##
# Download and unpack googletest at configure time
configure_file(cmake/googletest_download.cmake googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . 
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
execute_process(COMMAND ${CMAKE_COMMAND} --build . 
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# add google test src
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

##
## add library src
##
add_subdirectory(src)

##
## add test
##
add_subdirectory(test)
add_subdirectory(test_main)
add_subdirectory(test_measurement)
add_subdirectory(test_position_identify)
add_subdirectory(test_generate)
add_subdirectory(test_search)

##
## custom target
##
add_custom_target(gt
  COMMAND clear && ${CMAKE_BINARY_DIR}/test/test_mazelib
  DEPENDS test_mazelib mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(main
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_main/test_main
  DEPENDS test_main mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(meas
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_measurement/test_measurement
  DEPENDS test_measurement mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(pi
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_position_identify/test_position_identify
  DEPENDS test_position_identify mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(gen
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_generate/test_generate
  DEPENDS test_generate mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(search
  COMMAND clear && ${CMAKE_BINARY_DIR}/test_search/test_search
  DEPENDS test_search mazelib
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
