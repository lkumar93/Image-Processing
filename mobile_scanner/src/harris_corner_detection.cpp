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
#include "canny_edge_detection.h"

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////


float IndexT::distance(int j, int i)
{	
	return sqrt(pow((row-j),2)+pow((col-i),2));
}

CornerT harris_corner_detection(const Mat& input_image)
{

   Mat horizontal_kernel = Mat(3, 3, CV_32F, 0.0);
   Mat vertical_kernel = Mat(3, 3, CV_32F, 0.0);
   Mat image_gradient, image_gradient_direction,converted_image, nms_image, double_thresholded_image;

   horizontal_kernel.at<float>(0,0) = 1.0/9.0 ;
   horizontal_kernel.at<float>(0,1) = 0 ;
   horizontal_kernel.at<float>(0,2) = -1.0/9.0 ;

   horizontal_kernel.at<float>(1,0) = 2/9.0 ;
   horizontal_kernel.at<float>(1,1) = 0 ;
   horizontal_kernel.at<float>(1,2) = -2/9.0 ;

   horizontal_kernel.at<float>(2,0) = 1/9.0 ;
   horizontal_kernel.at<float>(2,1) = 0 ;
   horizontal_kernel.at<float>(2,2) = -1/9.0 ;

   vertical_kernel.at<float>(0,0) = 1/9.0 ;
   vertical_kernel.at<float>(0,1) = 2/9.0 ;
   vertical_kernel.at<float>(0,2) = 1/9.0 ;

   vertical_kernel.at<float>(1,0) = 0 ;
   vertical_kernel.at<float>(1,1) = 0 ;
   vertical_kernel.at<float>(1,2) = 0 ;

   vertical_kernel.at<float>(2,0) = -1/9.0 ;
   vertical_kernel.at<float>(2,1) = -2/9.0 ;
   vertical_kernel.at<float>(2,2) = -1/9.0 ;

   Mat horizontal_image = convolve(input_image, horizontal_kernel); 
   Mat vertical_image = convolve(input_image, vertical_kernel); 

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
	   horizontal_vertical_image .at<float>(j,i)  = horizontal_image.at<float>(j,i)*vertical_image.at<float>(j,i) ;
	   
       }

   GaussianBlur( horizontal_image_squared, horizontal_image_squared, Size(9,9), 1.4, 1.4, BORDER_DEFAULT );
   GaussianBlur( vertical_image_squared, vertical_image_squared, Size(9,9), 1.4, 1.4, BORDER_DEFAULT );
   GaussianBlur( horizontal_vertical_image, horizontal_vertical_image, Size(9,9), 1.4, 1.4, BORDER_DEFAULT );

   Mat harris_response= Mat(horizontal_image.rows, horizontal_image.cols, CV_32F, 0.0);
   Mat corners= Mat::zeros(horizontal_image.rows, horizontal_image.cols, CV_8UC1 );//Mat(horizontal_image.rows, horizontal_image.cols, CV_8UC1, 0);

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
    threshold_image(harris_response, corners, -0.25, false, true);
   //threshold_image(harris_response, corners, -1000000, false, true);

   if(corners.type() != CV_8UC1)
   	corners.convertTo(corners,CV_8UC1);

//   for(int j = 0; j < horizontal_image.rows ; j++)
//      for(int i = 0; i < horizontal_image.cols ; i++)
//      {
//	   if( harris_response.at<float>(j,i)  > THRESHOLD)
//		corners.at<uchar>(j,i) = 255;
//				
//      }

//    for(int j = 0; j < horizontal_image.rows ; j++)
//      for(int i = 0; i < horizontal_image.cols ; i++)
//      {

//	   int max = 0;
//	   for(int m = 0; m < size ; m++)
//		for(int n = 0; n < size; n++)
//		{
//		    int row = j+m-offset;
//		    int col = i+n-offset;
//		
//		    if ( corners.at<uchar>(row,col) > max && j!=row && i!=col )
//			max = corners.at<uchar>(row,col);
//		}

//	   if( corners.at<uchar>(j,i)  > max)
//		corners.at<uchar>(j,i) = 255;


//	}

   //Mat dilated_image =  morphological_filter(corners,3,false,true);

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

    

    for (std::vector<IndexT>::iterator it = corner_indices.begin() ; it != corner_indices.end(); ++it)
    {
	cout<<"index row = "<<it->row<<" , col="<<it->col<<endl;
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
   CornerData.corner_indices = corner_indices;
   CornerData.corner_image = image_with_corners;
//   namedWindow("Corners", WINDOW_AUTOSIZE);
//   imshow("Corners", corners2); 
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

