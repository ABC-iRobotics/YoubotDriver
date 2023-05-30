
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake) # add FindMatlab module

find_package(Matlab REQUIRED)

add_matlabexporter(matlabwrapper mexYouBot.cpp class_handle.hpp)
target_link_libraries(matlabwrapper manipulatormodul)


