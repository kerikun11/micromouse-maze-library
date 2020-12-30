# author: Ryotaro Onuki <kerikun11+github@gmail.com>
# date: 2020.12.29

# find Doxygen
find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
  message(WARNING "doxygen command couldn't be found! skipping...")
  RETURN()
endif()

# Doxygen Configuration
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
set(DOXYGEN_STRIP_FROM_PATH ${CMAKE_HOME_DIRECTORY})
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_ENABLE_PREPROCESSING YES)
set(DOXYGEN_MACRO_EXPANSION YES)
set(DOXYGEN_EXPAND_ONLY_PREDEF YES)
set(DOXYGEN_PREDEFINED __attribute__((__packed__)))
set(DOXYGEN_CREATE_SUBDIRS YES)

# make a custom target to generate docs
set(TARGET_NAME "docs")
doxygen_add_docs(${TARGET_NAME}
  ${CMAKE_HOME_DIRECTORY}/include
  ${CMAKE_HOME_DIRECTORY}/src
  ${CMAKE_HOME_DIRECTORY}/docs
  ${CMAKE_HOME_DIRECTORY}/README.md
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
# copy src files to html directory
add_custom_command(
  TARGET ${TARGET_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_HOME_DIRECTORY}/src ${CMAKE_CURRENT_BINARY_DIR}/html/src
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_HOME_DIRECTORY}/include ${CMAKE_CURRENT_BINARY_DIR}/html/include
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_HOME_DIRECTORY}/examples ${CMAKE_CURRENT_BINARY_DIR}/html/examples
)
