cmake_minimum_required(VERSION 2.8)
project( image_processing] )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories(../image_filters/include)
include_directories(../image_enhancement/include)
include_directories(../edge_detection/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(harris_corner_detection SHARED ../src/harris_corner_detection.cpp ../include/harris_corner_detection.h)

install(TARGETS harris_corner_detection DESTINATION bin)
install(FILES include/harris_corner_detection.h DESTINATION include)
