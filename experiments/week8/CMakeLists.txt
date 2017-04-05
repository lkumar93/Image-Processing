cmake_minimum_required(VERSION 2.8)
project( week8 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( dct ../src/dct.cpp )
target_link_libraries( dct ${OpenCV_LIBS} )
add_executable( deconvolution ../src/deconvolution.cpp )
target_link_libraries( deconvolution ${OpenCV_LIBS} )

