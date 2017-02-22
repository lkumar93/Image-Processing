cmake_minimum_required(VERSION 2.8)
project( week3 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( filter ../src/filter.cpp )
target_link_libraries( filter ${OpenCV_LIBS} )
add_executable( sharpening ../src/sharpening.cpp )
target_link_libraries( sharpening ${OpenCV_LIBS} )

