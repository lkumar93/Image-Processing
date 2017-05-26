//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS
// REQUIRED FOR ESTIMATING MOTION

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

#ifndef MOTION_ESTIMATION_H_
#define MOTION_ESTIMATION_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////

#define THRESHOLD 35000000000
#define PI 3.1416

#define SAD 0
#define SSD 1

///////////////////////////////////////////
//
//	NAMESPACES
//
///////////////////////////////////////////

using namespace cv;
using namespace std;

///////////////////////////////////////////
//
//	STRUCTS
//
///////////////////////////////////////////

struct MotionVector
{
	float vel_x;
	float vel_y;

	MotionVector() :vel_x(0.0),vel_y(0.0){};
};

///////////////////////////////////////////
//
//	CLASSES
//
///////////////////////////////////////////

class MotionField
{
	std::vector<MotionVector> motion_field;
        cv::Mat motion_field_image;
	int matching_criteria;
	int rows;
	int cols;

  public:

	int height;
	int width;
	MotionField(int num_rows = 0, int num_cols = 0, Mat previous_frame = Mat::zeros(1,1,CV_8UC3) );

	void set(int row, int col, const MotionVector& motion_vector);

	MotionVector& get(int row, int col);

	void print(int resolution=10);

	void quiver(const Mat& image, int resolution);

	std::vector<MotionVector> getMotionField();

	Mat getImage(int resolution=10);
};

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

MotionField estimateMotionByBlockMatching(const Mat& previousFrame, const Mat& currentFrame,int matching_criteria = SAD, int block_size = 8, int search_window_size = 32);

float mean_squared_error(const Mat& image_1, const Mat& image_2);

Mat compensateMotion(Mat previous_frame, MotionField motion_field);

Mat compute_prediction_error(const Mat& predicted_image, const Mat& target_image);

#endif //MOTION_ESTIMATION_H_
