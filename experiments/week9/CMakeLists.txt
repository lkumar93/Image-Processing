cmake_minimum_required(VERSION 2.8)
project( week9 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( harris_corner_detection ../src/harris_corner_detection.cpp )
target_link_libraries( harris_corner_detection ${OpenCV_LIBS} )
add_executable( adaptive_threshold ../src/adaptive_threshold.cpp )
target_link_libraries( adaptive_threshold ${OpenCV_LIBS} )

