## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.13

## add warnings in examples
## c.f. https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html#Warning-Options
add_compile_options(
  -Wall # This enables all the warnings about constructions that some users consider questionable, and that are easy to avoid (or modify to prevent the warning), even in conjunction with macros. 
  -Wcast-align # Warn whenever a pointer is cast such that the required alignment of the target is increased.
  -Wdouble-promotion # Give a warning when a value of type float is implicitly promoted to double.
  -Wextra # This enables some extra warning flags that are not enabled by -Wall. 
  -Wfatal-errors # This option causes the compiler to abort compilation on the first error occurred rather than trying to keep going and printing further error messages.
  -Wfloat-equal # Warn if floating-point values are used in equality comparisons.
  -Wundef # Warn if an undefined identifier is evaluated in an #if directive. Such identifiers are replaced with zero.
)

## add examples
add_subdirectory(contest)
add_subdirectory(generate)
add_subdirectory(load)
add_subdirectory(main)
add_subdirectory(meas)
add_subdirectory(pi)
add_subdirectory(search)

## add qt examples
add_subdirectory(qt_maze EXCLUDE_FROM_ALL)
add_subdirectory(qt_search EXCLUDE_FROM_ALL)
