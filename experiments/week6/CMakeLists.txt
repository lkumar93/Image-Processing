cmake_minimum_required(VERSION 2.8)
project( week4 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( deconvolution ../src/deconvolution.cpp )
target_link_libraries( deconvolution ${OpenCV_LIBS} )

