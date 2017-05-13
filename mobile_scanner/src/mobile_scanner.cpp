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

#define THRESHOLD_RATIO_H 0.24
#define THRESHOLD_RATIO_L 0.

using namespace cv;
using namespace std;

struct RansacT
{
	Mat best_perspective_transform;
	std::vector<int> best_four_matches;
};

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////


RansacT get_best_perspective_transform(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, std::vector<DMatch> matches, int iterations = 50000, float threshold = 0.175)
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


    RansacT data;
    data.best_perspective_transform = best_perspective_transform ;
    data.best_four_matches = best_index_vector;

  return data;
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

   if(input_image.cols > 480 || input_image.rows >640 )
     	rgb_image_resized = resize_image(input_image,input_image.cols/scale_factor,input_image.rows/scale_factor);

   else
   {
	rgb_image_resized = input_image.clone();
	scale_factor = 1;
   }

   grayscale_image =  convert_to_grayscale(rgb_image_resized);
   grayscale_image =	gaussian_filter(grayscale_image,3, 1.4);

   equalized_image = grayscale_image; 

   equalized_image = bilateral_filter(equalized_image, 5, 25, 15);

   if(equalized_image.type() != CV_8UC1)
	equalized_image.convertTo(equalized_image, CV_8UC1);

   equalized_image = sharpen(equalized_image,5, 1);

   canny_detected_edges = canny_edge_detection(equalized_image, THRESHOLD_RATIO_L, THRESHOLD_RATIO_H);

   detected_edges = canny_detected_edges;

   detected_edges =  morphological_filter( detected_edges, 5, true, false);
   detected_edges =  morphological_filter( detected_edges, 3, false, false);
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
		cout<<"Cannot process image with less than 4 corners"<<endl;
		return input_image;
  	 }
	 else
		cout<<"Image with "<<corner_data.corner_indices.size()<<" corners found"<<endl;

	 order_points(corner_data.corner_indices, ordered_points);

	 cout<<"Scaled Corner Points"<<endl;

	 for(int i = 0 ; i<4 ; i++)
	 {
		cout<<"row = "<<ordered_points[i].row*scale_factor<<" , col="<<ordered_points[i].col*scale_factor<<endl;
		circle( viz_corner, Point( ordered_points[i].col, ordered_points[i].row ), 10,  Scalar(0,255,0), 2, 8, 0 );
		ordered_points[i].row = cvFloor(ordered_points[i].row*scale_factor);
		ordered_points[i].col = cvFloor(ordered_points[i].col*scale_factor);
	 }	

	 int width1 = cvFloor(sqrt(pow(ordered_points[3].col - ordered_points[2].col,2) + pow(ordered_points[3].row - ordered_points[2].row,2)));
	 int width2 = cvFloor(sqrt(pow(ordered_points[1].col - ordered_points[0].col,2) + pow(ordered_points[1].row - ordered_points[0].row,2)));

	 int width = (width1 > width2) ? width1 : width2;

	 int height1 = cvFloor(sqrt(pow(ordered_points[1].col - ordered_points[2].col,2) + pow(ordered_points[1].row - ordered_points[2].row,2)));
	 int height2 = cvFloor(sqrt(pow(ordered_points[3].col - ordered_points[0].col,2) + pow(ordered_points[3].row - ordered_points[0].row,2)));

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

	 Mat pt =  perspective_transform(input_rectangle, output_rectangle);

	 warp_image(rgb_image_resized, warped_image, pt);

	 bilinearInterpolation(warped_image,vacant_pixel_value);

	 warped_grayscale_image = convert_to_grayscale(warped_image);

	 receipt = warped_grayscale_image;

	 threshold_image(gaussian_filter(warped_grayscale_image,3, 1.4), receipt, 5, false, true);

//  	 receipt =  morphological_filter( receipt, 3, false, false);
//  	 receipt =  morphological_filter( receipt, 3, true, false);
// 	 receipt =  morphological_filter( receipt, 3, false, false);

	  if(!image_registration)
	  {
		  namedWindow("Input", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/resized_input.jpg", rgb_image_resized );
		  imshow("Input",rgb_image_resized);

		  namedWindow("Image Enhancement", WINDOW_AUTOSIZE);
		   imwrite( "../results/document_scan/image_enhancement.jpg", equalized_image );
		  imshow("Image Enhancement",equalized_image);  

		  namedWindow("Canny Edge Detection", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/canny_edge_detection.jpg", canny_detected_edges );
		  imshow("Canny Edge Detection",canny_detected_edges);  

		  namedWindow("Morphological Operations", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/morphological_operation.jpg", detected_edges );
		  imshow("Morphological Operations",detected_edges);  

		  namedWindow("Contour Detection", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/contour_detection.jpg", viz_contour );
		  imshow("Contour Detection",viz_contour);

		  namedWindow("Corner Detection", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/corner_detection.jpg", viz_corner);
		  imshow("Corner Detection",viz_corner);

		  namedWindow("Warp with Perspective Transform", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/warped_image.jpg", warped_image );
		  imshow("Warp with Perspective Transform",warped_image);	  

		  namedWindow("Adaptive Thresholding", WINDOW_AUTOSIZE);
		  imwrite( "../results/document_scan/adaptive_thresholding.jpg", receipt );
		  imshow("Adaptive Thresholding",receipt);
	  }

	  cout<<"Width = "<<width<<std::endl;
	  cout<<"Height = "<<height<<std::endl;
   }
   return warped_image;
}

Mat background_subtraction(Mat image1, Mat image2)
{
	Mat thresholded_image1,thresholded_image2, subtracted_image1,subtracted_image2;
	Mat improved_image1 = sharpen(gaussian_filter(convert_to_grayscale(image1),5, 2),5,1);
	Mat improved_image2 = sharpen(gaussian_filter(convert_to_grayscale(image2),5, 1.4),5,1) ;
    
	threshold_image(improved_image1,thresholded_image1, 5, false, true);
	threshold_image(improved_image2,thresholded_image2, 5, false, true);

	subtracted_image1 = Mat::zeros(image1.rows,image1.cols,CV_8UC1);
	subtracted_image2 = Mat::zeros(image1.rows,image1.cols,CV_8UC1);

	for(int j = 0; j < image1.rows ; j++)
		for(int i = 0; i < image1.cols; i++)
		{
			if(thresholded_image1.at<uchar>(j,i) == thresholded_image2.at<uchar>(j,i) && thresholded_image1.at<uchar>(j,i) == 0)
				subtracted_image1.at<uchar>(j,i) = 255;
			else
				subtracted_image1.at<uchar>(j,i) = 0;
		}	

        subtracted_image1 =  morphological_filter( subtracted_image1, 5, true, false);

	for(int j = 0; j < image1.rows ; j++)
		for(int i = 0; i < image1.cols; i++)
		{
			if(subtracted_image1.at<uchar>(j,i) == thresholded_image2.at<uchar>(j,i) && thresholded_image2.at<uchar>(j,i) == 0 )
				subtracted_image2.at<uchar>(j,i) = 0;
			else
				subtracted_image2.at<uchar>(j,i) = 255;			
		}

	imshow("Thresholded Reference Image",thresholded_image1);
        imwrite( "../results/image_registration/thresholded_reference_image.jpg", thresholded_image1);

	imshow("Thresholded Scanned Image",thresholded_image2);
        imwrite( "../results/image_registration/thresholded_scanned_image.jpg", thresholded_image2);

	subtracted_image2 = morphological_filter( subtracted_image2, 3, false, false);
	subtracted_image2 =  median_filter(subtracted_image2,5);

	return subtracted_image2;
}


Mat foreground_addition(const Mat& image1, const Mat& image2, const Mat& foreground)
{
    Mat restored_image = image1.clone();

	for(int j = 10; j < image1.rows-10 ; j++)
		for(int i = 10; i < image1.cols-10; i++)
		{
			if(foreground.at<uchar>(j,i) == 0 )
			{
				if(image1.type() == CV_8UC3)
					restored_image.at<Vec3b>(j,i) = image2.at<Vec3b>(j,i);
				else
					restored_image.at<uchar>(j,i) = image2.at<uchar>(j,i);
			}
		}



   return restored_image;

}

Mat image_matching(Mat reference_image, Mat scanned_image)
{

    int size = 0;
    int count = 0;
    int max_iterations = 20;

    std::vector<DMatch> matches;

    vector<KeyPoint> keypoints1, keypoints2;

    while(size < 500 && count < max_iterations)
    {
	    Ptr<FeatureDetector> detector = ORB::create(5000 + count*500);
	    Ptr<DescriptorExtractor> extractor = ORB::create();

	    detector->detect(reference_image, keypoints1);
	    detector->detect(scanned_image, keypoints2);

	    Mat descriptors1,descriptors2;
	    extractor->compute(reference_image, keypoints1,descriptors1);
	    extractor->compute(scanned_image, keypoints2,descriptors2);

	    vector< vector<DMatch> > matches12, matches21;
	    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	    matcher->knnMatch( descriptors1, descriptors2, matches12, 2 );
	    matcher->knnMatch( descriptors2, descriptors1, matches21, 2 );

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
	size = better_matches.size();
	matches = better_matches;
	count++;
	cout<<"Feature Detection : Iteration ="<<count<<" , num features = "<<size<<endl;
   }

   Mat orb_reference_image = reference_image.clone();
   Mat orb_scanned_image = scanned_image.clone();

   for(int i=0; i<matches.size(); i++)
   {
	int col1 = keypoints1[matches[i].queryIdx].pt.x;
	int row1 = keypoints1[matches[i].queryIdx].pt.y;

	int col2 = keypoints2[matches[i].trainIdx].pt.x;
	int row2 = keypoints2[matches[i].trainIdx].pt.y;

	circle( orb_reference_image, Point( col1, row1), 10,  Scalar(0,255,0), 2, 8, 0 );
	circle( orb_scanned_image, Point( col2, row2), 10,  Scalar(0,255,0), 2, 8, 0 );
    }

   RansacT data = get_best_perspective_transform(keypoints1, keypoints2, matches);
   Mat best_perspective_transform = data.best_perspective_transform;

   vector<KeyPoint> best_keypoints1,best_keypoints2;
   std::vector<DMatch> best_matches;

   for(int i=0; i<4; i++)
   {
	DMatch match = matches[data.best_four_matches[i]];

        KeyPoint keypoint1 = keypoints1[match.queryIdx];
        KeyPoint keypoint2 = keypoints2[match.trainIdx];
	
	best_keypoints1.push_back(KeyPoint(keypoint1.pt,keypoint1.size));
	best_keypoints2.push_back(KeyPoint(keypoint2.pt,keypoint2.size));
	best_matches.push_back(DMatch(i, i, match.distance));
   }

   Mat best_match_image;
   drawMatches(reference_image, best_keypoints1, scanned_image, best_keypoints2, best_matches, best_match_image);

   Mat output_image = Mat::zeros(reference_image.rows,reference_image.cols,CV_8UC3);

   warp_image(scanned_image,output_image,best_perspective_transform);

   bilinearInterpolation(output_image);

   imshow("Better Matches Reference Image",orb_reference_image);
   imwrite( "../results/image_registration/orb_reference_image.jpg", orb_reference_image);

   imshow("Better Matches Scanned Image",orb_scanned_image);
   imwrite( "../results/image_registration/orb_scanned_image.jpg", orb_scanned_image);

   imshow("Best Four Matches RANSAC",best_match_image);
   imwrite( "../results/image_registration/best_four_matches_ransac.jpg", best_match_image);

   return output_image;

}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////
int main(int argc, char** argv )
{
    Mat image1, image2;

    int registration = 1;

    if(argc>=2)
   {
	    try
	    {
		registration = atoi(argv[1]);
	    }

	   catch (Exception e)
	   {
		//do nothing
	   }
   }

   
    if(registration == 1)
    {
	    image2 = imread( "../images/image_registration/scanned_image.jpg", 1 );
	    image1 = imread( "../images/image_registration/reference_image.jpg", 1);

	    if ( !image1.data || !image2.data)
	    {
		printf("No image data \n");
		return -1;
	    }

	    Mat scanned_image = image2;
	    Mat reference_image = scan_document(image1,true);
	    Mat warped_scanned_image = image_matching(reference_image,scanned_image);
	    Mat foreground = background_subtraction(reference_image, warped_scanned_image);
	    Mat restored_image = foreground_addition(reference_image, warped_scanned_image, foreground);
	    Mat thresholded_restored_image;
	    threshold_image(convert_to_grayscale(restored_image),thresholded_restored_image, 6, false, true);

	    imshow("Reference Image",reference_image);
            imwrite( "../results/image_registration/reference_image.jpg", reference_image);

	    imshow("Warped Scanned Image",warped_scanned_image);
            imwrite( "../results/image_registration/warped_scanned_image.jpg", warped_scanned_image);

	    imshow("Background Subtraction",foreground);
            imwrite( "../results/image_registration/background_subtraction.jpg", foreground);

	    imshow("Thresholded Restored Image",thresholded_restored_image);
    	    imwrite( "../results/image_registration/thresholded_restored_image.jpg", thresholded_restored_image);

	    imshow("Restored Image",restored_image);
   	    imwrite( "../results/image_registration/restored_image.jpg", restored_image);
    }
    else
    {

	    image1 = imread( "../images/document_scan/scanned_image.jpg", 1 );
	    if (!image1.data)
	    {
		printf("No image data \n");
		return -1;
	    }
	    scan_document(image1);

     }
    
    waitKey(0);

    return 0;
}

