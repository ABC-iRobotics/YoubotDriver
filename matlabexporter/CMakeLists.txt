
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake) # add FindMatlab module



find_package(Matlab REQUIRED)


add_matlabexporter(test mexAdd.cpp)
target_link_libraries(test lowlevelcontrol)


