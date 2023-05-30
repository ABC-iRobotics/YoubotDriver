file (GLOB SOURCE_FILES "*.cpp")
file (GLOB HEADER_FILES "*.hpp")

pybind11_add_module(pythonexporter 
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_link_libraries(pythonexporter PUBLIC)
set_target_properties(pythonexporter PROPERTIES FOLDER "Wrapper")
install(TARGETS pythonexporter
  COMPONENT python
  LIBRARY DESTINATION "Python"
  )

file(GLOB python_files "${CMAKE_CURRENT_SOURCE_DIR}/*.py")
file(COPY ${python_files} DESTINATION "${CMAKE_BINARY_DIR}/bin/Python")
install(FILES ${python_files} DESTINATION "Python")