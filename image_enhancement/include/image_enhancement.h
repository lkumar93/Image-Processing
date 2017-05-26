//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS 
// REQUIRED FOR ENHANCING IMAGES
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


#ifndef IMAGE_ENHANCEMENT_H_
#define IMAGE_ENHANCEMENT_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////

#define R_WEIGHT 0.2989
#define G_WEIGHT 0.5870
#define B_WEIGHT 0.1140

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

Mat convert_to_grayscale(const Mat& input_image);
void compute_histogram(const Mat& input_image, int histogram[]);
void display_histogram(int histogram[], const char* name);
void compute_cumulative_histogram(int histogram[], int cumulative_histogram[]);
int scale_histogram(int cumulative_histogram[],int scaled_histogram[], float scaling_factor);
Mat equalize_image(const Mat& input_image);
Mat log_transformation(const Mat& input_image, int transformation_constant);
Mat inverse_transformation(const Mat& input_image);
Mat gamma_correction(const Mat& input_image, int gamma);
Mat sharpen(const Mat& input_image, int kernel_size, float alpha);
void threshold_image(const Mat& input_image, Mat& thresholded_image, float threshold, bool inverse = false, bool adaptive = false);

#endif // IMAGE_ENHANCEMENT_H_

