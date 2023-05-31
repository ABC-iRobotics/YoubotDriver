file (GLOB SOURCE_FILES "*.cpp")
file (GLOB HEADER_FILES "*.hpp")

pybind11_add_module(youbotpython 
	${SOURCE_FILES}
	${HEADER_FILES}
)

target_link_libraries(youbotpython PUBLIC manipulatormodul)
set_target_properties(youbotpython PROPERTIES FOLDER "Wrapper")
install(TARGETS youbotpython
  COMPONENT python
  LIBRARY DESTINATION "Python"
  )

file(GLOB python_files "${CMAKE_CURRENT_SOURCE_DIR}/*.py")
file(COPY ${python_files} DESTINATION "${CMAKE_BINARY_DIR}/bin/Python")
install(FILES ${python_files} DESTINATION "Python")