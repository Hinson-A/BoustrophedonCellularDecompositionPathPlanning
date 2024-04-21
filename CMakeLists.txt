cmake_minimum_required(VERSION 3.5)
project(BCD_Planner)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)
include_directories(/usr/include/eigen3)

message(STATUS "Opencv_include_Path: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "Opencv_Library_Path: " ${OpenCV_LIBRARY})
# message(STATUS "Opencv_Library_Path: " ${OpenCV_LIBS})
message(STATUS "OpenCV_LIBRARY_DIRS: " ${OpenCV_LIBRARY_DIRS})


#include_directories(/usr/include/opencv4/)
include_directories(bcd_algorithm/include)

add_library(libPathPlanning 
    bcd_algorithm/src/bcd.cpp
    bcd_algorithm/src/common_data.cpp
    bcd_algorithm/src/map_sdk.cpp
    )
target_link_libraries(libPathPlanning ${OpenCV_LIBS} )

#add_executable(BCD_Planner main.cpp a-star.h )
add_executable(BCD_Planner main.cpp
#    bcd_algorithm/src/bcd.cpp
    )
target_link_libraries(BCD_Planner 
    ${OpenCV_LIBS}
    libPathPlanning
    )