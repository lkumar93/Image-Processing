cmake_minimum_required(VERSION 2.8)
project( image_processing )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(image_transformation SHARED ../src/image_transformation.cpp ../include/image_transformation.h)

install(TARGETS image_transformation DESTINATION bin)
install(FILES include/image_transformation.h DESTINATION include)


