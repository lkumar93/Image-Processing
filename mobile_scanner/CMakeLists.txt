cmake_minimum_required(VERSION 2.8)
project( mobile_scanner )
find_package( OpenCV REQUIRED )

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable( canny_edge_detection ../src/canny_edge_detection.cpp )
#target_link_libraries( canny_edge_detection ${OpenCV_LIBS} )

#add_executable( bilateral_filter ../src/bilateral_filter.cpp )
#target_link_libraries( bilateral_filter ${OpenCV_LIBS} )

#add_executable( morphological_filter ../src/morphological_filter.cpp )
#target_link_libraries( morphological_filter ${OpenCV_LIBS} )

add_library(image_transformation ../src/image_transformation.cpp ../include/image_transformation.h)
add_library(image_filters ../src/image_filters.cpp ../include/image_filters.h)
add_library(image_enhancement ../src/image_enhancement.cpp ../include/image_enhancement.h)
add_library(edge_detection ../src/edge_detection.cpp  ../include/edge_detection.h)
add_library(harris_corner_detection ../src/harris_corner_detection.cpp ../include/harris_corner_detection.h ../include/edge_detection.h ../src/edge_detection.cpp )


add_executable( mobile_scanner ../src/mobile_scanner.cpp )
target_link_libraries( mobile_scanner ${OpenCV_LIBS} image_filters image_enhancement image_transformation edge_detection  harris_corner_detection )

#add_executable( harris_corner_detection ../src/harris_corner_detection.cpp  )
#target_link_libraries( harris_corner_detection ${OpenCV_LIBS} filters)


