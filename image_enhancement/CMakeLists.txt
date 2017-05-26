cmake_minimum_required(VERSION 2.8)
project( image_processing )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories(../image_filters/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(image_enhancement SHARED ../src/image_enhancement.cpp ../include/image_enhancement.h)

install(TARGETS image_enhancement DESTINATION bin)
install(FILES include/image_enhancement.h DESTINATION include)
