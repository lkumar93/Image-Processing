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
#include <algorithm>  
#include "image_filters.h"
#include "image_enhancement.h"
#include "edge_detection.h"
#include "harris_corner_detection.h"
#include "image_transformation.h"
#include <opencv2/features2d.hpp> 

//#define THRESHOLD_RATIO_H 220
//#define THRESHOLD_RATIO_L 75

#define THRESHOLD_RATIO_H 0.24
#define THRESHOLD_RATIO_L 0.

#define USE_OPENCV false

using namespace cv;
using namespace std;

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////


Mat get_best_perspective_transform(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, std::vector<DMatch> matches, int iterations = 1000, int threshold = 2)
{

  std::vector<int> num_vector;
  // set some values:
  for (int i=0; i<matches.size(); ++i) num_vector.push_back(i); 

  int count = 0;

  int best_matched_points = 0;
  Mat best_perspective_transform;
  std::vector<int> best_index_vector;

  //DO RANSAC
  while(count<iterations)
  {
	std::random_shuffle ( num_vector.begin(), num_vector.end() );
	 	
	Point2f point1[4], point2[4];

	int index1 = num_vector[0];
	int index2 = num_vector[1];
	int index3 = num_vector[2];
	int index4 = num_vector[3];

	std::vector<int> index_vector;
	index_vector.push_back(index1);
	index_vector.push_back(index2);
	index_vector.push_back(index3);
	index_vector.push_back(index4);

	point1[0] = Point2f(keypoints1[matches[index1].queryIdx].pt.x, keypoints1[matches[index1].queryIdx].pt.y);
	point2[0] = Point2f(keypoints2[matches[index1].trainIdx].pt.x, keypoints2[matches[index1].trainIdx].pt.y);

	point1[1] = Point2f(keypoints1[matches[index2].queryIdx].pt.x, keypoints1[matches[index2].queryIdx].pt.y);
	point2[1] = Point2f(keypoints2[matches[index2].trainIdx].pt.x, keypoints2[matches[index2].trainIdx].pt.y);

	point1[2] = Point2f(keypoints1[matches[index3].queryIdx].pt.x, keypoints1[matches[index3].queryIdx].pt.y);
	point2[2] = Point2f(keypoints2[matches[index3].trainIdx].pt.x, keypoints2[matches[index3].trainIdx].pt.y);

	point1[3] = Point2f(keypoints1[matches[index4].queryIdx].pt.x, keypoints1[matches[index4].queryIdx].pt.y);
	point2[3] = Point2f(keypoints2[matches[index4].trainIdx].pt.x, keypoints2[matches[index4].trainIdx].pt.y);

	Mat pt = perspective_transform(point2, point1);

	int matched_points = 0;

	for (int i=0; i<matches.size(); ++i)
	{
		Mat B = Mat::ones( 3, 1, CV_32F );
		B.at<float>(0,0) = keypoints2[matches[i].trainIdx].pt.x;
		B.at<float>(1,0) = keypoints2[matches[i].trainIdx].pt.y;

		Mat A = Mat::ones( 3, 1, CV_32F );
		A.at<float>(0,0) = keypoints1[matches[i].queryIdx].pt.x;
		A.at<float>(1,0) = keypoints1[matches[i].queryIdx].pt.y;

		Mat Warped_B = pt*B;

		float euclidean_distance = sqrt(pow(A.at<float>(0,0)-Warped_B.at<float>(0,0),2)+pow(A.at<float>(1,0)-Warped_B.at<float>(1,0),2));

		if(euclidean_distance <= threshold)
			matched_points+=1;

	}

	if(matched_points > best_matched_points)
	{
		best_matched_points = matched_points;
		best_perspective_transform = pt;
		best_index_vector = index_vector;
	}

	count++;
  }

  return best_perspective_transform;
}

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

    Mat grayscale_image, equalized_image,detected_edges, canny_detected_edges,receipt,warped_image;

   float row_ratio = cvCeil(input_image.rows/640);
   float col_ratio = cvCeil(input_image.cols/480);

 
   int scale_factor = row_ratio>col_ratio?row_ratio:col_ratio;

   if(scale_factor < 1)
	scale_factor = 1;

   Mat rgb_image_resized = Mat::zeros( input_image.rows/scale_factor, input_image.cols/scale_factor, CV_8UC3 );

   if(USE_OPENCV)
   {
	    if(input_image.cols > 480 || input_image.rows >640 )
	    	resize(input_image, rgb_image_resized , rgb_image_resized.size(), 0, 0);

 	    else
	    {
		rgb_image_resized = input_image.clone();
		scale_factor = 1;
	    }

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
	    {
		rgb_image_resized = input_image.clone();
		scale_factor = 1;

	    }

	     grayscale_image =  convert_to_grayscale(rgb_image_resized);

//	     grayscale_image = equalized_image(grayscale_image);

	     grayscale_image =	gaussian_filter(grayscale_image,3, 1.4);

	     equalized_image = grayscale_image; // equalize_image(grayscale_image);

	     equalized_image = bilateral_filter(equalized_image, 5, 25, 15);

	     if(equalized_image.type() != CV_8UC1)
		equalized_image.convertTo(equalized_image, CV_8UC1);

	     equalized_image = sharpen(equalized_image,5, 1);

	     canny_detected_edges = canny_edge_detection(equalized_image, THRESHOLD_RATIO_L, THRESHOLD_RATIO_H);
    }

    detected_edges = canny_detected_edges;

    detected_edges =  morphological_filter( detected_edges, 5, true, false);
    detected_edges =  morphological_filter( detected_edges, 3, false, false);
    //detected_edges = median_filter(detected_edges,3);
    detected_edges =  morphological_filter( detected_edges, 3, false, false);
    detected_edges =  morphological_filter( detected_edges, 7, true, false);
    detected_edges =  morphological_filter( detected_edges, 3, true, false);
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

	  Mat image_of_contours =  Mat::zeros( rgb_image_resized.rows, rgb_image_resized.cols, CV_8UC1 );//rgb_image_resized;//Mat::zeros( detected_edges.size(), CV_8UC3 );;
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

	 CornerT corner_data = harris_corner_detection(image_of_contours,-0.25,30,0.03,4);

	 detected_corners = corner_data.corner_image;

	 IndexT ordered_points[4];

	 if(corner_data.corner_indices.size() < 4)
	 {
		  namedWindow("Contour Detection", WINDOW_AUTOSIZE);
		  imwrite( "../results/mobile_scanner/contour_detection.jpg", viz_contour );
		  imshow("Contour Detection",viz_contour);
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
		cout<<"row = "<<ordered_points[i].row*scale_factor<<" , col="<<ordered_points[i].col*scale_factor<<endl;
		circle( viz_corner, Point( ordered_points[i].col, ordered_points[i].row ), 10,  Scalar(0,255,0), 2, 8, 0 );
		ordered_points[i].row = int(ordered_points[i].row*scale_factor);
		ordered_points[i].col = int(ordered_points[i].col*scale_factor);
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

	 warped_image = Mat::ones(height,width,CV_8UC3)*vacant_pixel_value;

	rgb_image_resized = input_image.clone();
        
	 Mat warped_grayscale_image;

	 if(USE_OPENCV)   	
	 {	 
		  lambda = getPerspectiveTransform( input_rectangle, output_rectangle );

		  warpPerspective(rgb_image_resized,warped_image,lambda,warped_image.size() );

		  cvtColor(warped_image,warped_grayscale_image, CV_BGR2GRAY);
	  }

	  else
	  {

		   Mat pt =  perspective_transform(input_rectangle, output_rectangle);

		   warp_image(rgb_image_resized, warped_image, pt);

		   bilinearInterpolation(warped_image,vacant_pixel_value);

  		   warped_grayscale_image = convert_to_grayscale(warped_image);
			//warped_image.convertTo(warped_image, CV_8UC1);
	   }

	  receipt = warped_grayscale_image;

	  threshold_image(gaussian_filter(warped_grayscale_image,3, 1.4), receipt, 6.5, false, true);

  	  receipt =  morphological_filter( receipt, 3, false, false);
  	  receipt =  morphological_filter( receipt, 3, true, false);
 	  receipt =  morphological_filter( receipt, 3, false, false);

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
		  imwrite( "../results/mobile_scanner/corner_detection.jpg", viz_corner);
		  imshow("Corner Detection",viz_corner);

		  namedWindow("Warp with Perspective Transform", WINDOW_AUTOSIZE);
		  imwrite( "../results/mobile_scanner/warped_image.jpg", warped_image );
		  imshow("Warp with Perspective Transform",warped_image);	  

		  namedWindow("Adaptive Thresholding", WINDOW_AUTOSIZE);
		  imwrite( "../results/mobile_scanner/adaptive_thresholding.jpg", receipt );
		  imshow("Adaptive Thresholding",receipt);
	  }

	  else
	  {

		namedWindow("Registered Image", WINDOW_AUTOSIZE);
		imshow("Registered Image",warped_image);
	  }
	  cout<<"Width = "<<width<<std::endl;
	  cout<<"Height = "<<height<<std::endl;
   }
   return warped_image;
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////
int main(int argc, char** argv )
{
    Mat image1, image2;
    image1 = imread( "../images/receipt10.jpg", 1 );
    image2 = imread( "../images/registration_receipt3.jpg", 1);

    if ( !image1.data || !image2.data)
    {
        printf("No image data \n");
        return -1;
    }

    //cv::SiftFeatureDetector detector;

    Mat receipt1 = scan_document(image1);
    Mat receipt2 = scan_document(image2,true);

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> extractor = ORB::create();

    vector<KeyPoint> keypoints1, keypoints2;

    detector->detect(receipt1, keypoints1);
    detector->detect(receipt2, keypoints2);


    cout << "# keypoints of image1 :" << keypoints1.size() << endl;
    cout << "# keypoints of image2 :" << keypoints2.size() << endl;

    Mat descriptors1,descriptors2;
    extractor->compute(receipt1,keypoints1,descriptors1);
    extractor->compute(receipt2,keypoints2,descriptors2);

    cout << "Descriptors size :" << descriptors1.cols << ":"<< descriptors1.rows << endl;

    vector< vector<DMatch> > matches12, matches21;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch( descriptors1, descriptors2, matches12, 2 );
    matcher->knnMatch( descriptors2, descriptors1, matches21, 2 );

    cout << "Matches1-2:" << matches12.size() << endl;
    cout << "Matches2-1:" << matches21.size() << endl;

    std::vector<DMatch> good_matches1, good_matches2;

  // Ratio Test proposed by David Lowe 
    float ratio = 0.8;
    for(int i=0; i < matches12.size(); i++){
        if(matches12[i][0].distance < ratio * matches12[i][1].distance)
            good_matches1.push_back(matches12[i][0]);
    }

    for(int i=0; i < matches21.size(); i++){
        if(matches21[i][0].distance < ratio * matches21[i][1].distance)
            good_matches2.push_back(matches21[i][0]);
    }

    cout << "Good matches1:" << good_matches1.size() << endl;
    cout << "Good matches2:" << good_matches2.size() << endl;

    // Symmetric Test
    std::vector<DMatch> better_matches;
    for(int i=0; i<good_matches1.size(); i++){
        for(int j=0; j<good_matches2.size(); j++){
            if(good_matches1[i].queryIdx == good_matches2[j].trainIdx && good_matches2[j].queryIdx == good_matches1[i].trainIdx){
                better_matches.push_back(DMatch(good_matches1[i].queryIdx, good_matches1[i].trainIdx, good_matches1[i].distance));
                break;
            }
        }
	}

   cout << "Better matches:" << better_matches.size() << endl;


//    Mat output1 = receipt1.clone();
//    Mat output2 = receipt2.clone();
//    for(int i=0; i<better_matches.size(); i++)
//    {
//	int col1 = keypoints1[better_matches[i].queryIdx].pt.x;
//	int row1 = keypoints1[better_matches[i].queryIdx].pt.y;

//	int col2 = keypoints2[better_matches[i].trainIdx].pt.x;
//	int row2 = keypoints2[better_matches[i].trainIdx].pt.y;

//	circle( output1, Point( col1, row1), 10,  Scalar(0,255,0), 2, 8, 0 );
//	circle( output2, Point( col2, row2), 10,  Scalar(0,255,0), 2, 8, 0 );

//    }

   Mat best_perspective_transform = get_best_perspective_transform(keypoints1, keypoints2, better_matches);

   Mat output_image = Mat::zeros(receipt1.rows,receipt1.cols,CV_8UC3);

   warp_image(receipt2,output_image,best_perspective_transform);

    Mat output1 = receipt1.clone();
    Mat output2 = output_image.clone();
    for(int i=0; i<better_matches.size(); i++)
    {
	int col1 = keypoints1[better_matches[i].queryIdx].pt.x;
	int row1 = keypoints1[better_matches[i].queryIdx].pt.y;

	circle( output1, Point( col1, row1), 10,  Scalar(0,255,0), 2, 8, 0 );
	circle( output2, Point( col1, row1), 10,  Scalar(0,255,0), 2, 8, 0 );

    }


    Mat output;

   // show it on an image

//    drawMatches(receipt1, keypoints1, output_image, keypoints1, better_matches, output);
//    imshow("Output",output);
    imshow("Matches result1",output1);
    imshow("Matches result2",output2);
    
    waitKey(0);

    return 0;
}

