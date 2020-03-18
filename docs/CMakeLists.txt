find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_CONF_FILE "doxygen.conf")
  set(DOXYGEN_CONF_PATH ${CMAKE_CURRENT_BINARY_DIR}/${DOXYGEN_CONF_FILE})
  set(DOXYGEN_SOURCE_DIR ${CMAKE_SOURCE_DIR})
  set(DOXYGEN_TARGET "doc")

  configure_file(${DOXYGEN_CONF_FILE} ${DOXYGEN_CONF_PATH})

  add_custom_target(
    ${DOXYGEN_TARGET}
    ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONF_PATH}
    DEPENDS ${DOXYGEN_CONF_PATH})
else(DOXYGEN_FOUND)
  message(WARNING "doxygen binary couldn't be found")
endif()
