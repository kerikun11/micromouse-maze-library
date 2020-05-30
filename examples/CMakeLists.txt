## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.13

## make warning error in examples
add_compile_options(
  -Werror
  -Wundef
)

## add examples
add_subdirectory(search)
