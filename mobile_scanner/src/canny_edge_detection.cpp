//
// THIS FILE CONTAINS THE IMPLEMENTATION OF CANNY EDGE DETECTION
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

#include "morphological_filter.h"
#include "canny_edge_detection.h"

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

//Mat image_padding(const Mat& input_image, int offset)
//{
//   Mat padded_image = Mat(input_image.rows+2*offset, input_image.cols+2*offset, CV_32F, 0.0);

//   for(int j = 0; j < input_image.rows ; j++)
//        for(int i = 0; i < input_image.cols; i++)
//        {
//	    padded_image.at<float>(j+offset,i+offset) = input_image.at<uchar>(j,i);
//        }

//    return padded_image;
//}

//Mat image_depadding(const Mat& input_image, int offset)
//{
//   Mat depadded_image = Mat(input_image.rows-2*offset, input_image.cols-2*offset, CV_32F, 0.0);

//   for(int j = 0; j < input_image.rows-2*offset ; j++)
//        for(int i = 0; i < input_image.cols-2*offset; i++)
//        {
//	    depadded_image.at<float>(j,i) = input_image.at<float>(j+offset,i+offset);
//        }

//    return depadded_image;
//}

//Mat convolve(const Mat& input_image, const Mat& kernel)
//{
//  
//   int kernel_size = kernel.rows;

//   int offset;

//   if(kernel_size % 2 != 0)
//   {
//	offset = (kernel_size+1)/2 - 1;
//   }
//   else
//   {
//	offset = (kernel_size)/2 - 1;
//   }

//  Mat padded_image = image_padding(input_image, offset);

//  Mat flipped_kernel = Mat(kernel.rows, kernel.cols, CV_32F, 0.0);

//   for(int m = 0; m < kernel_size ; m++)
//	for(int n = 0; n < kernel_size; n++)
//	{
//		flipped_kernel.at<float>(m,n) = kernel.at<float>(kernel_size-m-1,kernel_size-n-1);
//	}

//  Mat convolved_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);	

//  float value = 0.0;
//  for(int j = offset; j < padded_image.rows - offset ; j++)
//       for(int i = offset; i < padded_image.cols - offset; i++)
//       {
//		  
//	   for(int m = 0; m < kernel_size ; m++)
//		for(int n = 0; n < kernel_size; n++)
//		{

//		    value += (padded_image.at<float>(j+m-offset,i+n-offset))*flipped_kernel.at<float>(m,n);

//		}

//	   convolved_image.at<float>(j,i)  = value;
//	   
//	   value = 0;	 
//       }

//  Mat depadded_image = image_depadding(convolved_image, offset);
//  
//  return depadded_image;
//}


Mat get_image_gradient(const Mat& input_image, const Mat& horizontal_image, const Mat& vertical_image)
{

    Mat filtered_image = Mat(input_image.rows, input_image.cols, CV_8UC1, 0.0);

    for(int j = 1; j < input_image.rows-1 ; j++)
       for(int i = 1; i < input_image.cols-1; i++)
       {
	    filtered_image.at<uchar>(j,i) = (( sqrt( pow(horizontal_image.at<float>(j,i),2) + pow(vertical_image.at<float>(j,i),2)))) ;
       }

   return filtered_image;

}

Mat get_image_gradient_direction(const Mat& input_image, const Mat& horizontal_image, const Mat& vertical_image)
{

   Mat filtered_image = Mat(input_image.rows, input_image.cols, CV_32F, 0.0);

   for(int j = 1; j < input_image.rows-1 ; j++)
       for(int i = 1; i < input_image.cols-1; i++)
       {
	    filtered_image.at<float>(j,i) =(float)(atan2( vertical_image.at<float>(j,i), horizontal_image.at<float>(j,i))*180/PI);
       }
 
   return filtered_image;

}

void threshold_image(const Mat& input_image, Mat& thresholded_image, float threshold, bool inverse=false, bool adaptive=false)
{

   
   thresholded_image = input_image.clone();
   int image_type = input_image.type();
   thresholded_image.convertTo(thresholded_image,CV_8UC1);
   Mat blurred_image;

   if(image_type == CV_32F)
	blurred_image = gaussian_filter(thresholded_image,5,1.4);
   else
   	blurred_image = gaussian_filter(input_image,11,1.8);
   blurred_image.convertTo(blurred_image,image_type);

   float c = threshold;
   for(int j = 1; j < input_image.rows-1; j++)
	for(int i =1; i < input_image.cols-1; i++)
	{
		if(image_type == CV_8UC1)
		{
			if(adaptive)
			{
				threshold =(int) saturate_cast<uchar>(blurred_image.at<uchar>(j,i) - c);
				
			}

			if (input_image.at<uchar>(j,i) >= threshold)
				if(inverse)
					thresholded_image.at<uchar>(j,i) = 0;
				else
					thresholded_image.at<uchar>(j,i) = 255;
			else
				if(inverse)
					thresholded_image.at<uchar>(j,i) = 255;
				else
					thresholded_image.at<uchar>(j,i) = 0;
		}
		else if(image_type == CV_32F)
		{
			if(adaptive)
			{
				threshold = (blurred_image.at<float>(j,i) - c);
			}

			if (input_image.at<float>(j,i) >= threshold)
				if(inverse)
					thresholded_image.at<uchar>(j,i) = 0;
				else
					thresholded_image.at<uchar>(j,i) = 255;
			else
				if(inverse)
					thresholded_image.at<uchar>(j,i) = 255;
				else
					thresholded_image.at<uchar>(j,i) = 0;

		}


	}

//   namedWindow("blur", WINDOW_AUTOSIZE);
//   imshow("blur", blurred_image); 
}

Mat non_maximum_suppression(const Mat& image_gradient,const Mat& gradient_direction, const Mat& horizontal_image, const Mat& vertical_image)
{
   Mat nms_image = image_gradient.clone();

   for(int j = 1; j < image_gradient.rows-1; j++)
	for(int i =1; i < image_gradient.cols-1; i++)
	{
		float angle = gradient_direction.at<float>(j,i);
		
		float top_elements[2];
		float bottom_elements[2];
		float ratio;

		float current_gradient = image_gradient.at<uchar>(j,i) ;

		float bottom_interpolation,top_interpolation;
		
		if(INTERPOLATION)
		{

			if( (angle >= 0 && angle <= 45) || (angle < -135 && angle >= -180))
				{
				  bottom_elements[0] = image_gradient.at<uchar>(j,i+1);
				  bottom_elements[1] = image_gradient.at<uchar>(j+1,i+1);

				  top_elements[0] = image_gradient.at<uchar>(j,i-1);
				  top_elements[1] = image_gradient.at<uchar>(j-1,i-1);
			
				  ratio = abs(vertical_image.at<float>(j,i)/current_gradient);

				  bottom_interpolation = (bottom_elements[1] - bottom_elements[0])*ratio +bottom_elements[0];
				  top_interpolation = (top_elements[1] - top_elements[0])*ratio +top_elements[0];			


				if(current_gradient < bottom_interpolation ||
				   current_gradient < top_interpolation )

					{nms_image.at<uchar>(j,i) = 0;}
				}

			else if( (angle > 45 && angle <= 90) || (angle < -90 && angle >= -135))

				{
				  bottom_elements[0] = image_gradient.at<uchar>(j+1,i);
				  bottom_elements[1] = image_gradient.at<uchar>(j+1,i+1);

				  top_elements[0] = image_gradient.at<uchar>(j-1,i);
				  top_elements[1] = image_gradient.at<uchar>(j-1,i-1);

				
				  ratio = abs(horizontal_image.at<float>(j,i)/current_gradient);

				  bottom_interpolation = (bottom_elements[1] - bottom_elements[0])*ratio +bottom_elements[0];
				  top_interpolation = (top_elements[1] - top_elements[0])*ratio +top_elements[0];			


				if(current_gradient < bottom_interpolation ||
				   current_gradient < top_interpolation )

					{nms_image.at<uchar>(j,i) = 0;	}
				}


			else if( (angle > 90 && angle <= 135) || (angle < -45 && angle >= -90))
				{
				  bottom_elements[0] = image_gradient.at<uchar>(j+1,i);
				  bottom_elements[1] = image_gradient.at<uchar>(j+1,i-1);

				  top_elements[0] = image_gradient.at<uchar>(j-1,i);
				  top_elements[1] = image_gradient.at<uchar>(j-1,i+1);

				
				  ratio = abs(horizontal_image.at<float>(j,i)/current_gradient);

				  bottom_interpolation = (bottom_elements[1] - bottom_elements[0])*ratio +bottom_elements[0];
				  top_interpolation = (top_elements[1] - top_elements[0])*ratio +top_elements[0];			


				if(current_gradient < bottom_interpolation ||
				   current_gradient < top_interpolation )

					{nms_image.at<uchar>(j,i) = 0;	}
				}

			else if( (angle > 135 && angle <= 180) || (angle < 0 && angle >= -45))

				{
				  bottom_elements[0] = image_gradient.at<uchar>(j,i-1);
				  bottom_elements[1] = image_gradient.at<uchar>(j+1,i-1);

				  top_elements[0] = image_gradient.at<uchar>(j,i+1);
				  top_elements[1] = image_gradient.at<uchar>(j-1,i+1);

				
				  ratio = abs(horizontal_image.at<float>(j,i)/current_gradient);

				  bottom_interpolation = (bottom_elements[1] - bottom_elements[0])*ratio +bottom_elements[0];
				  top_interpolation = (top_elements[1] - top_elements[0])*ratio +top_elements[0];			


				if(current_gradient < bottom_interpolation ||
				   current_gradient < top_interpolation )

					{nms_image.at<uchar>(j,i) = 0;		}
				}





		}

		else {
		
			if( (angle >= -22.5 && angle <= 22.5) || (angle < -157.5 && angle >= -180))

				if(image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j,i+1) ||
				   image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j,i-1) )

					{nms_image.at<uchar>(j,i) = 0;		}

			else if( (angle >= 22.5 && angle <= 67.5) || (angle < -112.5 && angle >= -157.5))

				if(image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j+1,i+1) ||
				   image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j-1,i-1) )

					{nms_image.at<uchar>(j,i) = 0;		}

			else if( (angle >= 67.5 && angle <= 112.5) || (angle < -67.5 && angle >= -112.5))

				if(image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j+1,i) ||
				   image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j-1,i) )

					{nms_image.at<uchar>(j,i) = 0;		}

			else if( (angle >= 112.5 && angle <= 157.5) || (angle < -22.5 && angle >= -67.5))

				if(image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j+1,i-1) ||
				   image_gradient.at<uchar>(j,i) < image_gradient.at<uchar>(j-1,i+1) )

					{nms_image.at<uchar>(j,i) = 0;		}

			

		}
		
			
	}


   return nms_image;

}

bool check_for_connected_strong_edges(int arr[],int size)
{
    int count = 0;
    for(int i = 0; i < size; i++)
	if(arr[i] == STRONG_EDGE)
//		count++;
//	if(count > 1)
		return true;	
 
    return false;   

}

Mat hysterisis(const Mat& input_image, int max_threshold, int min_threshold)
{

  Mat double_thresholded_image = input_image.clone() ;
  Mat edge_strength_image = input_image.clone();

  int size = input_image.rows * input_image.cols;	

  float strong_edges_row[size];

  int neighborhood[9];
  int k = 0;

   for(int j = 1; j < input_image.rows-1; j++)
	for(int i =1; i < input_image.cols-1; i++)
	{
		if (input_image.at<uchar>(j,i) >= max_threshold)
			edge_strength_image.at<uchar>(j,i) = STRONG_EDGE;


		else if (input_image.at<uchar>(j,i) > min_threshold && input_image.at<uchar>(j,i) < max_threshold)
			edge_strength_image.at<uchar>(j,i) = WEAK_EDGE ;		


		else
			edge_strength_image.at<uchar>(j,i) = NOT_EDGE;

	}

   for(int j = 1; j < input_image.rows-1; j++)
	for(int i =1; i < input_image.cols-1; i++)
	{
		if (edge_strength_image.at<uchar>(j,i) == WEAK_EDGE)
		    {
			k = 0;
			
			for(int m = -1; m < 2 ; m++)
			    for(int n = -1; n < 2; n++)
			    {		
				neighborhood[k] = edge_strength_image.at<uchar>(j+m,i+n) ;
				k++;
			    }

			if( check_for_connected_strong_edges( neighborhood, 9) )
			{
				double_thresholded_image.at<uchar>(j,i) = 255 ;
				edge_strength_image.at<uchar>(j,i) = STRONG_EDGE;
			}
			else
			{
				double_thresholded_image.at<uchar>(j,i) = 0 ;
				edge_strength_image.at<uchar>(j,i) = WEAK_EDGE;
			}
		    }

		else if (edge_strength_image.at<uchar>(j,i) == STRONG_EDGE)
			double_thresholded_image.at<uchar>(j,i) = 255 ; 

		else
			double_thresholded_image.at<uchar>(j,i) = 0 ;
	}


   return double_thresholded_image;


}

void sobel_filter(const Mat& input_image, Mat& image_gradient, Mat& image_gradient_direction, Mat& horizontal_image, Mat& vertical_image)
{

   Mat horizontal_kernel = Mat(3, 3, CV_32F, 0.0);
   Mat vertical_kernel = Mat(3, 3, CV_32F, 0.0);
   Mat nms_image, double_thresholded_image;

   horizontal_kernel.at<float>(0,0) = 1 ;
   horizontal_kernel.at<float>(0,1) = 0 ;
   horizontal_kernel.at<float>(0,2) = -1 ;

   horizontal_kernel.at<float>(1,0) = 2 ;
   horizontal_kernel.at<float>(1,1) = 0 ;
   horizontal_kernel.at<float>(1,2) = -2 ;

   horizontal_kernel.at<float>(2,0) = 1 ;
   horizontal_kernel.at<float>(2,1) = 0 ;
   horizontal_kernel.at<float>(2,2) = -1 ;

   vertical_kernel.at<float>(0,0) = 1 ;
   vertical_kernel.at<float>(0,1) = 2 ;
   vertical_kernel.at<float>(0,2) = 1 ;

   vertical_kernel.at<float>(1,0) = 0 ;
   vertical_kernel.at<float>(1,1) = 0 ;
   vertical_kernel.at<float>(1,2) = 0 ;

   vertical_kernel.at<float>(2,0) = -1 ;
   vertical_kernel.at<float>(2,1) = -2 ;
   vertical_kernel.at<float>(2,2) = -1 ;

   horizontal_image = convolve(input_image, horizontal_kernel); 
   vertical_image = convolve(input_image, vertical_kernel); 

   image_gradient = get_image_gradient(input_image,horizontal_image, vertical_image);

   image_gradient_direction = get_image_gradient_direction(input_image, horizontal_image, vertical_image);

}



Mat canny_edge_detection(const Mat& input_image, float threshold_l, float threshold_h)
{
   Mat gaussian_filtered_image, image_gradient,nms_image, double_thresholded_image;
   Mat gradient_direction, horizontal_image, vertical_image, sobel_filtered_image;
 
   gaussian_filtered_image = input_image.clone();

   GaussianBlur(gaussian_filtered_image,gaussian_filtered_image,Size(5,5), 1.4, 1.4, BORDER_DEFAULT);

   sobel_filter(gaussian_filtered_image, image_gradient,gradient_direction, horizontal_image, vertical_image);

   threshold_image(image_gradient,sobel_filtered_image, THRESHOLD_CANNY);

   nms_image = non_maximum_suppression(image_gradient,gradient_direction, horizontal_image, vertical_image) ;

   double min, max;

   cv::minMaxLoc(nms_image, &min, &max);

   int max_threshold = max*threshold_h;
   int min_threshold = max_threshold*threshold_l;

   double_thresholded_image = hysterisis( nms_image, max_threshold, min_threshold);

   return double_thresholded_image;


//   std::ostringstream path1, path2, path3 ;
//   path1<<"../results/canny_edge_detection/sobel_edge_detector"<<num<<".jpg";
//   path2<<"../results/canny_edge_detection/canny_edge_detector"<<num<<".jpg";
//   path3<<"../results/canny_edge_detection/nms_image"<<num<<".jpg";

//   namedWindow("Image Gradient", WINDOW_AUTOSIZE);
//   imwrite( path1.str(), image_gradient );
//   imshow("Sobel Image Gradient", image_gradient);

//   namedWindow("Sobel Filter ", WINDOW_AUTOSIZE);
//   imwrite( path1.str(), sobel_filtered_image );
//   imshow("Sobel Filter ", sobel_filtered_image);

//   namedWindow("Non Maximum Suppression", WINDOW_AUTOSIZE);
//   imwrite( path3.str(), nms_image );
//   imshow("Non Maximum Suppression", nms_image);

//   namedWindow("Hysterisis", WINDOW_AUTOSIZE);
//   imwrite(path2.str(), double_thresholded_image );
//   imshow("Hysterisis", double_thresholded_image);  

}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

//int main(int argc, char** argv )
//{
//    Mat grayscale_image1,grayscale_image2,grayscale_image3,grayscale_image4,grayscale_image5;

//    Mat image1 = imread( "../results/filter/equalized_image_1.jpg", 1 );
//    Mat image2 = imread( "../results/filter/gaussian_filtered_image_2.jpg", 1 );
//    Mat image3 = imread( "../results/filter/median_filtered_image_3.jpg", 1 );

//    //http://cnx.org/resources/4181ddf62e3a2047d36357f16180ce247b094532/plane_noise1.png
//    Mat image4 = imread( "../results/filter/median_filtered_image_4.jpg", 1 );

//    //en.wikipedia.org/wiki/File:Phase_correlation.png
//    Mat image5 = imread( "../results/filter/gaussian_filtered_image_5.jpg", 1 );

//    cv::Mat resized_image2 = cv::Mat(640, 1000, CV_8UC1, cv::Scalar(0, 0, 0)) ;

//    resize(image2,resized_image2,resized_image2.size(),0,0);

//    imwrite( "../results/canny_edge_detection/equalized_image2.jpg", resized_image2 );

//  if ( !image1.data || !resized_image2.data || !image3.data || !image4.data || !image5.data)
//    {
//        printf("No image data \n");
//        return -1;
//    }

//    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
//    cvtColor( resized_image2, grayscale_image2, CV_BGR2GRAY );
//    cvtColor( image3, grayscale_image3, CV_BGR2GRAY );
//    cvtColor( image4, grayscale_image4, CV_BGR2GRAY );
//    cvtColor( image5, grayscale_image5, CV_BGR2GRAY );

//    canny_edge_detection(grayscale_image1,1);
//    canny_edge_detection(grayscale_image2,2);
//    canny_edge_detection(grayscale_image3,3);
//    canny_edge_detection(grayscale_image4,4);
//    canny_edge_detection(grayscale_image5,5);

//    waitKey(0);

//    return 0;
//}

