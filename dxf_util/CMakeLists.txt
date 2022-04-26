cmake_minimum_required(VERSION 3.10)
project(dxf_util C CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(EIGEN_INCLUDE_DIR /usr/include/eigen3)
#find_package(Eigen REQUIRED)

# create the library
file(GLOB_RECURSE ${PROJECT_NAME}_src src/*.cpp src/*.c src/*.cc)
file(GLOB_RECURSE ${PROJECT_NAME}_dxfsrc src/dxflib/*.cpp src/dxflib/*.c src/dxflib/*.cc)

include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/include/dxf_util/ ${CMAKE_CURRENT_SOURCE_DIR}/include/dxf_util/nlohmann/ ${EIGEN_INCLUDE_DIR})
#link_directories(include ${EIGEN_INCLUDE_DIR})
add_library(MyLibs SHARED ${${PROJECT_NAME}_src} ${${PROJECT_NAME}_dxfsrc})

add_executable(geotest src/maotest.cpp)
target_link_libraries(geotest MyLibs)
