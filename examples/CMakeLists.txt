## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.13

## make warning error in examples
add_compile_options(
  -Werror
  -Wundef
)

## add examples
add_subdirectory(contest)
add_subdirectory(generate)
add_subdirectory(load)
add_subdirectory(main)
add_subdirectory(meas)
add_subdirectory(pi)
add_subdirectory(search)
