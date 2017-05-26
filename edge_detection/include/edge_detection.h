//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS 
// REQUIRED FOR VARIOUS EDGE DETECTORS
//
// COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE
//
// AUTHOR : LAKSHMAN KUMAR
// AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
// EMAIL : LKUMAR93@TERPMAIL.UMD.EDU
// LINKEDIN : WWW.LINKEDIN.COM/IN/LAKSHMANKUMAR1993
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THE GPLv3 LICENSE
// THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF
// THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS 
// PROHIBITED.
// 
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//

#ifndef EDGE_DETECTION_H_
#define EDGE_DETECTION_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////

#define THRESHOLD_CANNY 35

#define INTERPOLATION true

#define STRONG_EDGE 2
#define WEAK_EDGE 1
#define NOT_EDGE 0

///////////////////////////////////////////
//
//	NAMESPACE
//
///////////////////////////////////////////

using namespace cv;

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

Mat get_image_gradient(const Mat& input_image, const Mat& horizontal_image, const Mat& vertical_image);

Mat get_image_gradient_direction(const Mat& input_image, const Mat& horizontal_image, const Mat& vertical_image);

Mat non_maximum_suppression(const Mat& image_gradient,const Mat& gradient_direction, const Mat& horizontal_image, const Mat& vertical_image);

bool check_for_connected_strong_edges(int arr[],int size);

Mat hysterisis(const Mat& input_image, int max_threshold, int min_threshold);

void sobel_edge_detection(const Mat& input_image, Mat& image_gradient, Mat& image_gradient_direction, Mat& horizontal_image, Mat& vertical_image);

void prewitt_edge_detection(const Mat& input_image, Mat& image_gradient, Mat& image_gradient_direction, Mat& horizontal_image, Mat& vertical_image);

Mat canny_edge_detection(const Mat& input_image, float threshold_l, float threshold_h);

#endif // EDGE_DETECTION_H_

