cmake_minimum_required(VERSION 2.8.9)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-m64")
SET(CMAKE_C_FLAGS "-m64")
SET(CMAKE_EXE_LINKER_FLAGS "-m64")
SET(CMAKE_MODULE_LINKER_FLAGS "-m64")
SET(CMAKE_SHARED_LINKER_FLAGS "-m64")
project (simplereedssheppcarplanner)

# Depends on OMPL
find_package(ompl REQUIRED base)

# Depends on Eigen. REQUIRED!
find_package (Eigen3 REQUIRED)

include_directories( ${OMPL_INCLUDE_DIRS} )

include_directories( src/include )
file(GLOB SOURCES "src/*.cpp")
add_library(simplereedssheppcarplanner SHARED ${SOURCES})

target_link_libraries(simplereedssheppcarplanner ${OMPL_LIBRARIES} Eigen3::Eigen)

#Use "sudo make install" to apply
install(TARGETS simplereedssheppcarplanner DESTINATION /usr/local/lib)
