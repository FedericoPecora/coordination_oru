cmake_minimum_required(VERSION 2.8.9)
set (CMAKE_CXX_FLAGS "-m64")
project (simplereedssheppcarplanner)

#Depends on OMPL and MRPT
find_package(OMPL)
find_package(MRPT) 
include_directories( ${OMPL_INCLUDE_DIRS} )
include_directories( ${MRPT_INCLUDE_DIRS} )
include_directories( src/include )
file(GLOB SOURCES "src/*.cpp")
add_library(simplereedssheppcarplanner SHARED ${SOURCES})
target_link_libraries(simplereedssheppcarplanner ${OMPL_LIBRARIES} ${MRPT_LIBRARIES})

#Use "sudo make install" to apply
install(TARGETS simplereedssheppcarplanner DESTINATION /usr/local/lib)
