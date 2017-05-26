cmake_minimum_required(VERSION 2.8)
project( image_processing )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories(../image_filters/include)
include_directories(../image_enhancement/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(edge_detection SHARED ../src/edge_detection.cpp  ../include/edge_detection.h)

install(TARGETS edge_detection DESTINATION bin)
install(FILES include/edge_detection.h DESTINATION include)
