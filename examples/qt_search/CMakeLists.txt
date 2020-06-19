## author: Ryotaro Onuki <kerikun11+github@gmail.com>
## date: 2020.05.14

find_package(Qt5 COMPONENTS Widgets)
if(NOT Qt5_FOUND)
  message(WARNING "Qt5 Not Found. skipping...")
  RETURN()
endif()

# set config
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# glob files
file(GLOB_RECURSE UI_FILES *.ui)
file(GLOB_RECURSE CODE_FILES *.cpp)

# ui
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# target
set(TARGET_NAME example_qt_search)
add_executable(${TARGET_NAME}
  ${UI_HEADERS}
  ${CODE_FILES}
)
target_include_directories(${TARGET_NAME}
  PRIVATE ${CMAKE_HOME_DIRECTORY}/include # mazelib
)
target_link_libraries(${TARGET_NAME}
  Qt5::Widgets
  mazelib
)
## qt_search
add_custom_target(qt_search
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS ${TARGET_NAME} mazelib
  USES_TERMINAL
)
