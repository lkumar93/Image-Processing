cmake_minimum_required(VERSION 2.8)
project( image_processing )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories(../image_transformation/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(motion_estimation SHARED ../src/motion_estimation.cpp ../include/motion_estimation.h)

install(TARGETS motion_estimation DESTINATION bin)
install(FILES include/motion_estimation.h DESTINATION include)	
