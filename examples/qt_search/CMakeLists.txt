## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.14

## import qt package
find_package(Qt5 COMPONENTS Widgets)
if(NOT Qt5_FOUND)
  message(WARNING "Qt5 Not Found. skipping...")
  RETURN()
endif()

## set config
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## glob files
file(GLOB_RECURSE UI_FILES *.ui)
file(GLOB_RECURSE CODE_FILES *.cpp)

## qt ui
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

## give a name
set(CUSTOM_TARGET_NAME "qt_search")
set(TARGET_NAME example_${CUSTOM_TARGET_NAME})
## make a executable
add_executable(${TARGET_NAME}
  ${UI_HEADERS}
  ${CODE_FILES}
)
target_link_libraries(${TARGET_NAME}
  ${MICROMOUSE_MAZE_LIBRARY}
  Qt5::Widgets
)
## make a custom target to run example
add_custom_target(${CUSTOM_TARGET_NAME}
  COMMAND ${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  USES_TERMINAL
)
