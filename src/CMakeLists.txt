## author:  Ryotaro Onuki <kerikun11+github@gmail.com>
## date:    2020.12.26

## make a static library
set(TARGET_NAME "mazelib")
file(GLOB SRCS *.cpp)
add_library(${TARGET_NAME} STATIC ${SRCS})
target_include_directories(${TARGET_NAME}
  PUBLIC ${CMAKE_HOME_DIRECTORY}/include
)

## more warnnings
target_compile_options(${TARGET_NAME} PRIVATE
  -Wall # Enables a big group of common warnings - see https://clang.llvm.org/docs/DiagnosticsReference.html#wall
  -Wextra # Enables key warnings like -Wunused-parameter and -Wsign-compare - see https://clang.llvm.org/docs/DiagnosticsReference.html#wextra
  -Wundef # Warn if an undefined identifier is evaluated in an #if directive. Such identifiers are replaced with zero.
  -Wdouble-promotion # Give a warning when a value of type float is implicitly promoted to double.
)
