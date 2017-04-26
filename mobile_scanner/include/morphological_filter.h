//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS OF 
// THE MORPHOLOGICAL FILTER IMPLEMENTATION
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


#ifndef MORPHOLOGICAL_FILTER_H_
#define MORPHOLOGICAL_FILTER_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>

#define PI 3.14159
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

void merge(int A[ ] , int start, int mid, int end);
void merge_sort (int A[ ] , int start , int end );

Mat image_padding(const Mat& input_image, int offset);

Mat image_depadding(const Mat& input_image, int offset);

Mat convolve(const Mat& input_image, const Mat& kernel);

Mat median_filter(const Mat& input_image, int kernel_size);

Mat morphological_filter(const Mat& input_image, int kernel_size,bool max,bool ignore_center_pixel);

Mat mean_filter(const Mat& input_image, int kernel_size);

Mat gaussian_filter(const Mat& input_image, int kernel_size, float sigma);

#endif // MORPHOLOGICAL_FILTER_H_

