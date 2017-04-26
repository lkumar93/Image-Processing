//
// THIS HEADER FILE CONTAINS THE DECLARATION OF THE VARIABLES AND FUNCTIONS OF 
// THE HARRIS CORNER DETECTION IMPLEMENTATION
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

#ifndef HARRIS_CORNER_DETECTION_H_
#define HARRIS_CORNER_DETECTION_H_

///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>
#include "morphological_filter.h"

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////


//#define THRESHOLD 1000000000//
#define THRESHOLD 35000000000 //
#define DISTANCE_THRESHOLD 10

///////////////////////////////////////////
//
//	NAMESPACE
//
///////////////////////////////////////////

using namespace cv;
using namespace std;

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////


struct IndexT
{
	int col;
	int row;

	float distance(int j, int i);
};

struct CornerT
{
	std::vector<IndexT> corner_indices;
	Mat corner_image;
};



CornerT harris_corner_detection(const Mat& input_image);


#endif // HARRIS_CORNER_DETECTION_H_

