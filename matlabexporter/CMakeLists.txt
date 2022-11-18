
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake) # add FindMatlab module

find_package(Matlab REQUIRED)

add_matlabexporter(matlabexporter mexAdd.cpp)
target_link_libraries(matlabexporter lowlevelcontrol)


