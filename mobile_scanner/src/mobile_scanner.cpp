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
#include "image_transformation.h"

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

	if(sum < min_sum)
	{
		min_sum = sum ;
		ordered_points[0] = *it;	
	}

	if(diff < min_diff)
	{
		min_diff = diff ;
		ordered_points[1] = *it;
	}

	if(diff > max_diff)
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


 Mat scan_document(const Mat& input_image, bool image_registration = false)
{

    Mat grayscale_image, equalized_image,detected_edges, canny_detected_edges,receipt;

    int scale_factor = 4.0;

    Mat rgb_image_resized = Mat::zeros( input_image.rows/scale_factor, input_image.cols/scale_factor, CV_8UC3 );

    float row_ratio = scale_factor;
    float col_ratio = scale_factor;

   if(USE_OPENCV)
   {
	    if(input_image.cols > 480 || input_image.rows >640 )
	    	resize(input_image, rgb_image_resized , rgb_image_resized.size(), 0, 0);

 	    else
		rgb_image_resized = input_image.clone();

	    cvtColor( rgb_image_resized , grayscale_image, CV_BGR2GRAY );

	    equalizeHist(grayscale_image,grayscale_image);

	    bilateralFilter(grayscale_image, equalized_image, 5, 25, 20);

	    Canny( equalized_image, canny_detected_edges, 75, 220, 3 );
    }

    else
    {
	     if(input_image.cols > 480 || input_image.rows >640 )
	     	rgb_image_resized = resize_image(input_image,input_image.cols/scale_factor,input_image.rows/scale_factor);

 	     else
		rgb_image_resized = input_image.clone();

	     grayscale_image =  convert_to_grayscale(rgb_image_resized);

	     grayscale_image =	gaussian_filter(grayscale_image,3, 1.4);

	     equalized_image = grayscale_image; // equalize_image(grayscale_image);

	     equalized_image = bilateral_filter(equalized_image, 5, 25, 15);

	     if(equalized_image.type() != CV_8UC1)
		equalized_image.convertTo(equalized_image, CV_8UC1);

	     equalized_image = sharpen(equalized_image,5, 0.1);

	     canny_detected_edges = canny_edge_detection(equalized_image, THRESHOLD_RATIO_L, THRESHOLD_RATIO_H);
    }
   
    detected_edges = canny_detected_edges;

    detected_edges =  morphological_filter( detected_edges, 5, true, false);
    detected_edges =  morphological_filter( detected_edges, 3, false, false);
    //detected_edges = median_filter(detected_edges,3);
    detected_edges =  morphological_filter( detected_edges, 3, false, false);
    detected_edges =  morphological_filter( detected_edges, 7, true, false);
    detected_edges =  morphological_filter( detected_edges, 5, false, false);
    detected_edges =  morphological_filter( detected_edges, 3, false, false);

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

	  Mat image_of_contours =  Mat::zeros( input_image.rows/scale_factor, input_image.cols/scale_factor, CV_8UC1 );//rgb_image_resized;//Mat::zeros( detected_edges.size(), CV_8UC3 );;
	  Mat viz_corner = rgb_image_resized.clone();
	  Mat viz_contour =  rgb_image_resized.clone();

 	  RNG rng(12345);

	  // Find contour with largest area
	  for( int i = 0; i < number_of_contours ; i++ )
	  { 
      		cv::approxPolyDP(cv::Mat(contours[i]), contours[i], cv::arcLength(cv::Mat(contours[i]), true)*0.05, true);
		current_area = contourArea(contours[i]);
		current_number_of_vertices = contours[i].size();
	
		if ( current_area > previous_area )
		{
			largest_contour_area = current_area;
			largest_contour_id = i;
			number_of_vertices = current_number_of_vertices;	
			previous_area = current_area;	
		}

	  }

       	 Scalar color = Scalar( 255);
       	 drawContours( image_of_contours , contours, largest_contour_id, color, 2, 8, hierarchy, 0, Point());
      	 drawContours( viz_contour , contours, largest_contour_id, Scalar(0, 255,0),2, 8, hierarchy, 0, Point());	

	 Mat detected_corners;

	 CornerT corner_data = harris_corner_detection(image_of_contours,-0.25,10,0.03,4);

	 detected_corners = corner_data.corner_image;

	 IndexT ordered_points[4];

	 if(corner_data.corner_indices.size() < 4)
	 {
		cout<<"Cannot process image with less than 4 corners"<<endl;
		return input_image;
  	 }
	 else
	 {
		cout<<"Image with "<<corner_data.corner_indices.size()<<" corners found"<<endl;
	  }

	 order_points(corner_data.corner_indices, ordered_points);

	 for(int i = 0 ; i<4 ; i++)
	 {
		cout<<"row = "<<ordered_points[i].row*row_ratio<<" , col="<<ordered_points[i].col*col_ratio<<endl;
		circle( viz_corner, Point( ordered_points[i].col, ordered_points[i].row ), 10,  Scalar(0,255,0), 2, 8, 0 );
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

	 int vacant_pixel_value = 0;

	 Mat warped_image = Mat::ones(height,width,CV_8UC1)*vacant_pixel_value;

	rgb_image_resized = input_image.clone();

	 if(USE_OPENCV)   	
	 {	 
		  lambda = getPerspectiveTransform( input_rectangle, output_rectangle );

		  warpPerspective(rgb_image_resized,warped_image,lambda,warped_image.size() );

		  cvtColor(warped_image,warped_image, CV_BGR2GRAY);
	  }

	  else
	  {

		   Mat pt =  perspective_transform(input_rectangle, output_rectangle);

		   warp_image(convert_to_grayscale(rgb_image_resized), warped_image, pt);

		   bilinearInterpolation(warped_image,vacant_pixel_value);

		   if(warped_image.type()!=CV_8UC1)
			warped_image.convertTo(warped_image, CV_8UC1);
	   }

	  receipt = warped_image;

	  threshold_image(warped_image, receipt, 6, false, true);

	  if(!image_registration)
	  {
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
		  imwrite( "../results/mobile_scanner/contour_detection.jpg", viz_contour );
		  imshow("Contour Detection",viz_contour);

		  namedWindow("Corner Detection", WINDOW_AUTOSIZE);
		  imwrite( "../results/mobile_scanner/corner_detection.jpg", detected_corners );
		  imshow("Corner Detection",detected_corners);

		  namedWindow("Warp with Perspective Transform", WINDOW_AUTOSIZE);
		  imwrite( "../results/mobile_scanner/warped_image.jpg", warped_image );
		  imshow("Warp with Perspective Transform",warped_image);	  

		  namedWindow("Adaptive Thresholding", WINDOW_AUTOSIZE);
		  imwrite( "../results/mobile_scanner/adaptive_thresholding.jpg", receipt );
		  imshow("Adaptive Thresholding",receipt);
	  }

	  cout<<"Width = "<<width<<std::endl;
	  cout<<"Height = "<<height<<std::endl;

   }


   return receipt;
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

    Mat receipt1 = scan_document(rgb_image);
    
    waitKey(0);

    return 0;
}

