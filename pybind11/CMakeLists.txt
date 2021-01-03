# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2021.01.03

# find pybind11
find_package(pybind11)
if(NOT pybind11_FOUND)
  message(WARNING "pybind11 not found in your environment! skipping...")
  RETURN()
endif()

# make a python module
set(MODULE_NAME "MazeLib")
file(GLOB SRC_FILES
  ${PROJECT_SOURCE_DIR}/src/*.cpp # also build src files with pybind11 options
  *.cpp
)
pybind11_add_module(${MODULE_NAME} ${SRC_FILES})
target_include_directories(${MODULE_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

# find python interpreter
find_package(Python3)
if(NOT Python3_FOUND)
  message(WARNING "Python3 not found in your environment! skipping...")
  RETURN()
endif()

# make a custom target to plot
set(CUSTOM_TARGET_NAME "pybind11_plot")
add_custom_target(${CUSTOM_TARGET_NAME}
  COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
  ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot.py
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS ${MODULE_NAME}
  USES_TERMINAL
)
