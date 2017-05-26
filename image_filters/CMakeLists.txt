cmake_minimum_required(VERSION 2.8)
project( image_processing )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(image_filters SHARED ../src/image_filters.cpp ../include/image_filters.h)

install(TARGETS image_filters DESTINATION bin)
install(FILES include/image_filters.h DESTINATION include)


