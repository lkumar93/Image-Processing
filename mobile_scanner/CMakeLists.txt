cmake_minimum_required(VERSION 2.8)
project( mobile_scanner )
find_package( OpenCV REQUIRED )
LINK_DIRECTORIES(/usr/local/bin/)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( mobile_scanner ../src/mobile_scanner.cpp )
target_link_libraries( mobile_scanner ${OpenCV_LIBS} image_filters image_enhancement image_transformation edge_detection  harris_corner_detection )
