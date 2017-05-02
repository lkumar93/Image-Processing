cmake_minimum_required(VERSION 2.8)
project( week9 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( motion_estimation ../src/motion_estimation.cpp )
target_link_libraries( motion_estimation ${OpenCV_LIBS} )

