//
// THIS FILE CONTAINS THE IMPLEMENTATION OF A MOBILE DOCUMENT SCANNER
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


///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>
#include "image_filters.h"
#include "image_enhancement.h"
#include "edge_detection.h"
#include "harris_corner_detection.h"

//#define THRESHOLD_RATIO_H 220
//#define THRESHOLD_RATIO_L 75

#define THRESHOLD_RATIO_H 0.24
#define THRESHOLD_RATIO_L 0.2

#define USE_OPENCV false

using namespace cv;
using namespace std;

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////


//Order the points as Top_Left, Top_Right, Bottom_Right, Bottom_Left
void order_points(std::vector<IndexT> corner_indices, IndexT ordered_points[])
{
    int max_sum = 0;
    int min_sum = 100000;
    int max_diff = -100000;
    int min_diff = 100000;


    for (std::vector<IndexT>::iterator it = corner_indices.begin() ; it != corner_indices.end(); ++it)
    {
	int sum = it->row + it->col;
	int diff = it->row - it->col;

	if(sum > max_sum)
	{
		max_sum = sum ;
		ordered_points[2] = *it;
	}

	else if(sum < min_sum)
	{
		min_sum = sum ;
		ordered_points[0] = *it;
	}

	if(diff < min_diff )
	{
		min_diff = diff ;
		ordered_points[1] = *it;
	}

	else if(diff > max_diff )
	{
		max_diff = diff ;
		ordered_points[3] = *it;
	}

    }
	
   cout<<" Ordered Points : Top_Left, Top_Right, Bottom_Right, Bottom_Left"<<endl;

   for(int i = 0 ; i<4 ; i++)
   {
		cout<<"row = "<<ordered_points[i].row<<" , col="<<ordered_points[i].col<<endl;
   }	

}


Mat perspective_transform(Point2f corner_points[], Point2f reference_points[])
{
	Mat A = Mat::ones( 3, 3, CV_32F );
	Mat B = Mat::ones( 3, 1, CV_32F );

	A.at<float>(0,0) = corner_points[0].x;
	A.at<float>(1,0) = corner_points[0].y;

	A.at<float>(0,1) = corner_points[1].x;
	A.at<float>(1,1) = corner_points[1].y;

	A.at<float>(0,2) = corner_points[2].x;
	A.at<float>(1,2) = corner_points[2].y;

	B.at<float>(0,0) = corner_points[3].x;
	B.at<float>(1,0) = corner_points[3].y;

	Mat C = Mat::ones( 3, 1, CV_32F );
	C = A.inv()*B;

	Mat M = Mat::ones( 3, 3, CV_32F );

	M.at<float>(0,0) = C.at<float>(0,0)*corner_points[0].x;
	M.at<float>(1,0) =  C.at<float>(0,0)*corner_points[0].y;
	M.at<float>(2,0) =  C.at<float>(0,0);

	M.at<float>(0,1) =  C.at<float>(1,0)* corner_points[1].x;
	M.at<float>(1,1) =  C.at<float>(1,0)*corner_points[1].y;
	M.at<float>(2,1) =  C.at<float>(1,0);

	M.at<float>(0,2) =  C.at<float>(2,0)*corner_points[2].x;
	M.at<float>(1,2) =  C.at<float>(2,0)*corner_points[2].y;
	M.at<float>(2,2) =  C.at<float>(2,0);

	Mat D = Mat::ones( 3, 3, CV_32F );
	Mat E = Mat::ones( 3, 1, CV_32F );

	D.at<float>(0,0) = reference_points[0].x;
	D.at<float>(1,0) = reference_points[0].y;

	D.at<float>(0,1) = reference_points[1].x;
	D.at<float>(1,1) = reference_points[1].y;

	D.at<float>(0,2) = reference_points[2].x;
	D.at<float>(1,2) = reference_points[2].y;

	E.at<float>(0,0) = reference_points[3].x;
	E.at<float>(1,0) = reference_points[3].y;

	Mat F = Mat::ones( 3, 1, CV_32F );
	F = D.inv()*E;

	Mat N = Mat::ones( 3, 3	, CV_32F );

	N.at<float>(0,0) = F.at<float>(0,0)*reference_points[0].x;
	N.at<float>(1,0) = F.at<float>(0,0)*reference_points[0].y;
	N.at<float>(2,0) = F.at<float>(0,0);

	N.at<float>(0,1) = F.at<float>(1,0)*reference_points[1].x;
	N.at<float>(1,1) = F.at<float>(1,0)*reference_points[1].y;
	N.at<float>(2,1) = F.at<float>(1,0);

	N.at<float>(0,2) = F.at<float>(2,0)*reference_points[2].x;
	N.at<float>(1,2) = F.at<float>(2,0)*reference_points[2].y;
	N.at<float>(2,2) = F.at<float>(2,0);


	Mat perspective_transform= Mat::ones( 3, 3, CV_32F );
	perspective_transform = N*M.inv();

	return perspective_transform;
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////
int main(int argc, char** argv )
{
    Mat rgb_image, grayscale_image, equalized_image,detected_edges, canny_detected_edges;
    rgb_image = imread( "../images/receipt.jpg", 1 );

    if ( !rgb_image.data )
    {
        printf("No image data \n");
        return -1;
    }

    Mat rgb_image_resized = Mat::zeros( 640, 480, CV_8UC3 );

    float row_ratio = 1;//rgb_image.rows/640.0;
    float col_ratio = 1;//rgb_image.cols/480.0;

    resize(rgb_image, rgb_image_resized , rgb_image_resized.size(), 0, 0);
   
    rgb_image = rgb_image_resized;

   if(USE_OPENCV)
   {
    cvtColor( rgb_image_resized , grayscale_image, CV_BGR2GRAY );

    equalizeHist(grayscale_image,grayscale_image);

    bilateralFilter(grayscale_image, equalized_image, 5, 25, 20);

    Canny( equalized_image, canny_detected_edges, 75, 220, 3 );
    }

    else
    {
     grayscale_image =  convert_to_grayscale(rgb_image_resized);

     equalized_image = equalize_image(grayscale_image);

     equalized_image = bilateral_filter(equalized_image, 5, 15, 10);

     if(equalized_image.type() != CV_8UC1)
	equalized_image.convertTo(equalized_image, CV_8UC1);

     equalized_image = sharpen(equalized_image,5, 0.1);

     canny_detected_edges = canny_edge_detection(equalized_image, THRESHOLD_RATIO_L, THRESHOLD_RATIO_H);

    }

    detected_edges =  morphological_filter( canny_detected_edges, 5, true, false);
  
    if(detected_edges.type() != CV_8UC1)
	    detected_edges.convertTo(detected_edges, CV_8UC1);

    detected_edges =  morphological_filter( detected_edges, 3, false, false);

    if(detected_edges.type() != CV_8UC1)
	    detected_edges.convertTo(detected_edges, CV_8UC1);

    detected_edges =  morphological_filter( detected_edges, 3, false, false);

    if(detected_edges.type() != CV_8UC1)
	    detected_edges.convertTo(detected_edges, CV_8UC1);

    detected_edges =  morphological_filter( detected_edges, 3, true, false);

    if(detected_edges.type() != CV_8UC1)
	    detected_edges.convertTo(detected_edges, CV_8UC1);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Point> largest_contour;
    vector<Point> rectangle;

   findContours( detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

   int number_of_contours = contours.size();
  
   if (number_of_contours > 0 )
   {
	  float previous_area = 0.0;
	  float current_area = 0.0;
	  int current_number_of_vertices = 0;
	  int largest_contour_id = 0;
	  float largest_contour_length;
	  float largest_contour_area;
	  int largest_contour_center_x;
	  int largest_contour_center_y;
	  int number_of_vertices;          

	  Mat image_of_contours =  Mat::zeros( 640, 480, CV_8UC1 );//rgb_image_resized;//Mat::zeros( detected_edges.size(), CV_8UC3 );;

 	  RNG rng(12345);

	  // Find contour with largest area
	  for( int i = 0; i < number_of_contours ; i++ )
	     { 

      		cv::approxPolyDP(cv::Mat(contours[i]), contours[i], cv::arcLength(cv::Mat(contours[i]), true)*0.05, true);
		current_area = contourArea(contours[i]);
		current_number_of_vertices = contours[i].size();
	
		if ( current_number_of_vertices == 4 && current_area > previous_area )
		{
			//
			largest_contour_area = current_area;
			largest_contour_id = i;
			number_of_vertices = current_number_of_vertices;	
			previous_area = current_area;	
		}
	   // Draw contours
//       	   Scalar color = Scalar( 255);//, 255, 0 );
//       	   drawContours( image_of_contours , contours, i, color, 2, 8, hierarchy, 0, Point() );

	     }

       	   Scalar color = Scalar( 255);//, 255, 0 );
       	   drawContours( image_of_contours , contours, largest_contour_id, color, 2, 8, hierarchy, 0, Point());	

	 Mat detected_corners;
//	  /// Detecting corners
//	  cornerHarris( image_of_contours, detected_corners, 2, 3, 0.04, BORDER_DEFAULT );

//	  /// Normalizing
//	  normalize(  detected_corners,  detected_corners, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
//	  convertScaleAbs(  detected_corners,  detected_corners );


////	  /// Drawing a circle around corners

	  CornerT corner_data = harris_corner_detection(image_of_contours);

//    waitKey(0);

//    return 0;
	  detected_corners = corner_data.corner_image;

	  IndexT ordered_points[4];

	  order_points(corner_data.corner_indices, ordered_points);

	   for(int i = 0 ; i<4 ; i++)
	   {
			cout<<"row = "<<ordered_points[i].row<<" , col="<<ordered_points[i].col<<endl;
			ordered_points[i].row = int(ordered_points[i].row*row_ratio);
			ordered_points[i].col = int(ordered_points[i].col*col_ratio);
	   }	

	   int width1 = int(sqrt(pow(ordered_points[3].col - ordered_points[2].col,2) + pow(ordered_points[3].row - ordered_points[2].row,2)));
	   int width2 = int(sqrt(pow(ordered_points[1].col - ordered_points[0].col,2) + pow(ordered_points[1].row - ordered_points[0].row,2)));

	   int width = (width1 > width2) ? width1 : width2;

	   int height1 = int(sqrt(pow(ordered_points[1].col - ordered_points[2].col,2) + pow(ordered_points[1].row - ordered_points[2].row,2)));
	   int height2 = int(sqrt(pow(ordered_points[3].col - ordered_points[0].col,2) + pow(ordered_points[3].row - ordered_points[0].row,2)));

	   int height = (height1 > height2) ? height1 : height2;

	   Point2f input_rectangle[4], output_rectangle[4];

	   input_rectangle[0] = Point2f(ordered_points[0].col, ordered_points[0].row);
	   input_rectangle[1] = Point2f(ordered_points[1].col, ordered_points[1].row);
	   input_rectangle[2] = Point2f(ordered_points[2].col, ordered_points[2].row);
	   input_rectangle[3] = Point2f(ordered_points[3].col, ordered_points[3].row);

	   output_rectangle[0] = Point2f(0,0);
	   output_rectangle[1] = Point2f(width-1,0);
	   output_rectangle[2] = Point2f(width-1,height-1);
	   output_rectangle[3] = Point2f(0,height-1);

           Mat lambda( 2, 4, CV_32FC1 );

	   Mat warped_image = Mat::ones(height,width,CV_8UC1)*0;


	  if(USE_OPENCV)   	
	  {
	 
		  lambda = getPerspectiveTransform( input_rectangle, output_rectangle );

		  warpPerspective(rgb_image,warped_image,lambda,warped_image.size() );

		  cvtColor(warped_image,warped_image, CV_BGR2GRAY);

	  }

	  else
	  {

	   Mat pt =  perspective_transform(input_rectangle, output_rectangle);

	   Mat transformed_point = Mat::ones(3,1,CV_32F);
	   Mat given_point = Mat::ones(3,1,CV_32F); 

	   for( int j = 0; j <  grayscale_image.rows ; j++ )
   	      for( int i = 0; i <  grayscale_image.cols; i++ )
		{
			given_point.at<float>(0,0) = i;
			given_point.at<float>(1,0) = j;

			transformed_point = pt*given_point;

			int x = cvFloor(transformed_point.at<float>(1,0)/transformed_point.at<float>(2,0));
			int y = cvFloor(transformed_point.at<float>(0,0)/transformed_point.at<float>(2,0));

			if(x >= 0 && x < height && y >=0 && y< width)
				warped_image.at<uchar>(x,y) = grayscale_image.at<uchar>(j,i);

		}

	   warped_image = median_filter(warped_image, 3);

	   if(warped_image.type()!=CV_8UC1)
		warped_image.convertTo(warped_image, CV_8UC1);
	   }

	  Mat receipt = warped_image;

	  threshold_image(warped_image, receipt, 6, false, true);

	  namedWindow("Input", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/resized_input.jpg", rgb_image_resized );
	  imshow("Input",rgb_image_resized);

	  namedWindow("Image Enhancement", WINDOW_AUTOSIZE);
           imwrite( "../results/mobile_scanner/image_enhancement.jpg", equalized_image );
	  imshow("Image Enhancement",equalized_image);  

	  namedWindow("Canny Edge Detection", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/canny_edge_detection.jpg", canny_detected_edges );
	  imshow("Canny Edge Detection",canny_detected_edges);  

	  namedWindow("Morphological Operations", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/morphological_operation.jpg", detected_edges );
	  imshow("Morphological Operations",detected_edges);  

	  namedWindow("Contour Detection", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/contour_detection.jpg", image_of_contours );
	  imshow("Contour Detection",image_of_contours);

	  namedWindow("Corner Detection", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/corner_detection.jpg", detected_corners );
	  imshow("Corner Detection",detected_corners);

	  namedWindow("Warp with Perspective Transform", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/warped_image.jpg", warped_image );
	  imshow("Warp with Perspective Transform",warped_image);	  

	  namedWindow("Adaptive Thresholding", WINDOW_AUTOSIZE);
          imwrite( "../results/mobile_scanner/adaptive_thresholding.jpg", receipt );
	  imshow("Adaptive Thresholding",receipt);

	  cout<<"Width = "<<width<<std::endl;
	  cout<<"Height = "<<height<<std::endl;

   }
    
    waitKey(0);

    return 0;
}

