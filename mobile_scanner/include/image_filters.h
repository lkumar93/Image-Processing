//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS
// REQUIRED FOR FILTERING IMAGES
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


#ifndef IMAGE_FILTERS_H_
#define IMAGE_FILTERS_H_

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

Mat bilateral_convolve(const Mat& input_image, const Mat& gaussian_kernel, float sigma_r);

Mat bilateral_filter(const Mat& input_image, int kernel_size, float sigma_g, float sigma_r);

Mat image_padding_for_dft(const Mat& input_image);

Mat add_noise(const Mat& input_image, int std_dev);

Mat get_gaussian_blur_kernel(int kernel_size, float sigma);

Mat get_motion_blur_kernel(int kernel_size);

Mat fourier_transform(const Mat& padded_image);

Mat power_spectrum(const Mat& input_image);

Mat wiener_filter(const Mat& noisy_image, const Mat& signal_spectrum, Mat kernel, int kernel_size, float threshold, float std_dev);

Mat inverse_filter(const Mat& noisy_image, Mat kernel, int kernel_size, float std_dev, float threshold = 0.2, bool pseudo_inverse = false);

Mat pad_kernel(cv::Size size,const Mat& kernel, int kernel_size);

#endif // IMAGE_FILTERS_H_

