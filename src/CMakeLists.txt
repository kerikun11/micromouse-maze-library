## author: Ryotaro Onuki <kerikun11+github@gmail.com>
file(GLOB SRCS *.cpp)
add_library(mazelib STATIC ${SRCS})
target_include_directories(mazelib PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_compile_options(mazelib PRIVATE
  -Werror
  -Wall
  -Wextra
  -Wundef
  -fno-common
  -Wdouble-promotion
)
