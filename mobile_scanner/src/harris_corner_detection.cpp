//
// THIS FILE CONTAINS THE IMPLEMENTATION OF HARRIS CORNER DETECTION
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

#include "harris_corner_detection.h"
#include "image_enhancement.h"
#include "image_filters.h"
#include "edge_detection.h"

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////


float IndexT::distance(int j, int i)
{	
	return sqrt(pow((row-j),2)+pow((col-i),2));
}

bool IndexT::operator!=(const IndexT& rhs)
{
	if(this->row != rhs.row && this->col != rhs.col)
		return true;
	else
		return false;
}

bool IndexT::operator==(const IndexT& rhs)
{
	if(this->row == rhs.row && this->col == rhs.col)
		return true;
	else
		return false;
}

CornerT harris_corner_detection(const Mat& input_image, float threshold, int iterations,float step_size, int min_corners_num)
{

   Mat horizontal_kernel = Mat(3, 3, CV_32F, 0.0);
   Mat vertical_kernel = Mat(3, 3, CV_32F, 0.0);
   Mat image_gradient, image_gradient_direction,converted_image, horizontal_image, vertical_image;

   sobel_edge_detection(input_image, image_gradient, image_gradient_direction, horizontal_image, vertical_image);

   if(horizontal_image.type() != CV_32F)
	horizontal_image.convertTo(horizontal_image,CV_32F);

   if(vertical_image.type() != CV_32F)
	vertical_image.convertTo(vertical_image,CV_32F);

   Mat horizontal_image_squared = Mat(horizontal_image.rows, horizontal_image.cols, CV_32F, 0.0);
   Mat vertical_image_squared  = Mat(horizontal_image.rows, horizontal_image.cols, CV_32F, 0.0);
   Mat horizontal_vertical_image = Mat(horizontal_image.rows, horizontal_image.cols, CV_32F, 0.0);

   Mat combined_image = Mat(horizontal_image.rows, horizontal_image.cols, CV_32F, 0.0);

    for(int j = 0; j < horizontal_image.rows ; j++)
       for(int i = 0; i < horizontal_image.cols ; i++)
       {
	   combined_image.at<float>(j,i) = horizontal_image.at<float>(j,i) + vertical_image.at<float>(j,i);
	   horizontal_image_squared.at<float>(j,i) = pow( horizontal_image.at<float>(j,i), 2);
	   vertical_image_squared.at<float>(j,i)  = pow( vertical_image.at<float>(j,i), 2);
	   horizontal_vertical_image.at<float>(j,i)  = horizontal_image.at<float>(j,i)*vertical_image.at<float>(j,i) ;
	   
       }

   horizontal_image_squared = gaussian_filter(horizontal_image_squared, 9, 1.4);
   vertical_image_squared = gaussian_filter(vertical_image_squared, 9, 1.4);
   horizontal_vertical_image = gaussian_filter(horizontal_vertical_image, 9, 1.4);

   if(horizontal_image_squared.type() != CV_32F)
	horizontal_image_squared.convertTo(horizontal_image_squared,CV_32F);

   if(vertical_image_squared.type() != CV_32F)
	vertical_image_squared.convertTo(vertical_image_squared,CV_32F);

   if(horizontal_vertical_image.type() != CV_32F)
	horizontal_vertical_image.convertTo(horizontal_vertical_image,CV_32F);

   //GaussianBlur( horizontal_image_squared, horizontal_image_squared, Size(9,9), 1.4, 1.4, BORDER_DEFAULT );
   //GaussianBlur( vertical_image_squared, vertical_image_squared, Size(9,9), 1.4, 1.4, BORDER_DEFAULT );
   //GaussianBlur( horizontal_vertical_image, horizontal_vertical_image, Size(9,9), 1.4, 1.4, BORDER_DEFAULT );

   Mat harris_response= Mat(horizontal_image.rows, horizontal_image.cols, CV_32F, 0.0);
   Mat corners = Mat::zeros(horizontal_image.rows, horizontal_image.cols, CV_8UC1 );//Mat(horizontal_image.rows, horizontal_image.cols, CV_8UC1, 0);

   float k = 0.04;

   for(int j = 0; j < horizontal_image.rows ; j++)
      for(int i = 0; i < horizontal_image.cols ; i++)
      {
	   //R = Det(H) - k(Trace(H))^2;
   
	   //cout<<harris_response.at<float>(j,i);
	   harris_response.at<float>(j,i) = horizontal_image_squared.at<float>(j,i)*vertical_image_squared.at<float>(j,i) -pow(horizontal_vertical_image.at<float>(j,i),2) 
					    - k*(pow(horizontal_image_squared.at<float>(j,i)+vertical_image_squared.at<float>(j,i),2));
		
      }

   Mat corners2 = Mat::zeros(horizontal_image.rows, horizontal_image.cols, CV_8UC1 );
   
   int size = 5;

   int offset = (size+1)/2 - 1;

    normalize(harris_response, harris_response, 1,0,NORM_MINMAX);

   std::vector<IndexT> best_corner_indices;

   if(min_corners_num == 0)
   { 
	   threshold_image(harris_response, corners, threshold, false, true);

	   if(corners.type() != CV_8UC1)
	   	corners.convertTo(corners,CV_8UC1);

	   std::vector<IndexT> corner_indices;

	   for(int j = 0; j < horizontal_image.rows ; j++)
	      for(int i = 0; i < horizontal_image.cols ; i++)
	      {
		   IndexT current_index;
		   current_index.row = j;
		   current_index.col = i;

		   if( corners.at<uchar>(j,i)  > 200)
			if(corner_indices.empty())
				corner_indices.push_back(current_index);
			else
			{
				 bool corner_neighbor = false;
				 for (std::vector<IndexT>::iterator it = corner_indices.begin() ; it != corner_indices.end(); ++it)
				 {
					if(it->distance(j,i) < DISTANCE_THRESHOLD)
						corner_neighbor = true;
				 }	

				 if(!corner_neighbor)
					corner_indices.push_back(current_index);			
			}			
	      }

	  best_corner_indices = corner_indices;
   }

   else
   {

	int min_corners = 10000;

	int count = 0;
	
	float local_threshold = threshold - ( step_size*iterations/2.0);

	int corner_size = min_corners;

	while(count < iterations)
	{

	   corners = Mat::zeros(horizontal_image.rows, horizontal_image.cols, CV_8UC1 );

	   threshold_image(harris_response, corners, local_threshold, false, true);

	   if(corners.type() != CV_8UC1)
	   	corners.convertTo(corners,CV_8UC1);

	   std::vector<IndexT> corner_indices;

	   for(int j = 0; j < horizontal_image.rows ; j++)
	      for(int i = 0; i < horizontal_image.cols ; i++)
	      {
		   IndexT current_index;
		   current_index.row = j;
		   current_index.col = i;

		   if( corners.at<uchar>(j,i)  > 200)
			if(corner_indices.empty())
				corner_indices.push_back(current_index);
			else
			{
				 bool corner_neighbor = false;
				 for (std::vector<IndexT>::iterator it = corner_indices.begin() ; it != corner_indices.end(); ++it)
				 {
					if(it->distance(j,i) < DISTANCE_THRESHOLD)
						corner_neighbor = true;
				 }	

				 if(!corner_neighbor)
					corner_indices.push_back(current_index);			
			}			
	      }
	
	   if(corner_indices.size()<min_corners && corner_indices.size() >= min_corners_num )
	   {
		best_corner_indices = corner_indices;
		min_corners = corner_indices.size();

		if(corner_indices.size() == min_corners_num)
		{
			break;
		}

	   }



	   count++;

//	   if(corner_size-corner_indices.size() >= 0.0)
//	  	 local_threshold-=step_size;

//	   else
		local_threshold+=step_size;

	   std::cout<<"Iteration = "<<count<< " ,num corners = "<<corner_indices.size()<<" ,local threshold ="<< local_threshold<<endl;

	   corner_size = corner_indices.size();

	} 
    }


    cout<<"HARRIS CORNER DETECTION"<<endl;

    for (std::vector<IndexT>::iterator it = best_corner_indices.begin() ; it != best_corner_indices.end(); ++it)
    {
	cout<<"corner at row = "<<it->row<<" , col="<<it->col<<endl;
	corners2.at<uchar>(it->row,it->col) = 255;
	
    }	

   Mat image_with_corners = input_image.clone();

    for( int j = 0; j <  corners2.rows ; j++ )
     { for( int i = 0; i <  corners2.cols; i++ )
	  {
	    if( (int)  corners2.at<uchar>(j,i) == 255 )
	      {
	       circle(  image_with_corners, Point( i, j ), 10,  Scalar(150), 2, 8, 0 );
	      }
	  }
     }


   CornerT CornerData;
   CornerData.corner_indices = best_corner_indices;
   CornerData.corner_image = image_with_corners;
   return CornerData; 
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

//int main(int argc, char** argv )
//{
//    Mat grayscale_image;

//    Mat image = imread( "../images/contour2.png", 1 );

//    if ( !image.data )
//     {
//        printf("No image data \n");
//        return -1;
//     }

//    Mat image_resized = Mat::zeros( 640, 480, CV_8UC3 );

//    resize(image, image_resized , image_resized.size(), 0, 0);

//    cvtColor( image_resized, grayscale_image, CV_BGR2GRAY );

//    Mat corners = harris_corner_detection(grayscale_image);

//    namedWindow("Receipt", WINDOW_AUTOSIZE);
//    imshow("Receipt", corners); 
//	
//    waitKey(0);

//    return 0;
//}

