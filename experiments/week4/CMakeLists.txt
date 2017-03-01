cmake_minimum_required(VERSION 2.8)
project( week4 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( bilateral_filter ../src/bilateral_filter.cpp )
target_link_libraries( bilateral_filter ${OpenCV_LIBS} )
add_executable( morphological_filter ../src/morphological_filter.cpp )
target_link_libraries( morphological_filter ${OpenCV_LIBS} )

