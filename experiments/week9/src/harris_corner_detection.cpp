#include <iostream>
#include <opencv2/opencv.hpp>

#define THRESHOLD 35000000000
#define DISTANCE_THRESHOLD 8

using namespace cv;
using namespace std;

struct IndexT
{
	int col;
	int row;

	float distance(int j, int i)
	{
		return sqrt(pow((row-j),2)+pow((col-i),2));
	}
};


Mat image_padding(const Mat& input_image, int offset)
{
   Mat padded_image = Mat(input_image.rows+2*offset, input_image.cols+2*offset, CV_32F, 0.0);

   for(int j = 0; j < input_image.rows ; j++)
        for(int i = 0; i < input_image.cols; i++)
        {
	    padded_image.at<float>(j+offset,i+offset) = input_image.at<uchar>(j,i);
        }

    return padded_image;
}

Mat image_depadding(const Mat& input_image, int offset)
{
   Mat depadded_image = Mat(input_image.rows-2*offset, input_image.cols-2*offset, CV_32F, 0.0);

   for(int j = 0; j < input_image.rows-2*offset ; j++)
        for(int i = 0; i < input_image.cols-2*offset; i++)
        {
	    depadded_image.at<float>(j,i) = input_image.at<float>(j+offset,i+offset);
        }

    return depadded_image;
}

Mat convolve(const Mat& input_image, const Mat& kernel)
{
  
   int kernel_size = kernel.rows;

   int offset;

   if(kernel_size % 2 != 0)
   {
	offset = (kernel_size+1)/2 - 1;
   }
   else
   {
	offset = (kernel_size)/2 - 1;
   }

  Mat padded_image = image_padding(input_image, offset);

  Mat flipped_kernel = Mat(kernel.rows, kernel.cols, CV_32F, 0.0);

   for(int m = 0; m < kernel_size ; m++)
	for(int n = 0; n < kernel_size; n++)
	{
		flipped_kernel.at<float>(m,n) = kernel.at<float>(kernel_size-m-1,kernel_size-n-1);
	}

  Mat convolved_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);	

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
		  
	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{

		    value += (padded_image.at<float>(j+m-offset,i+n-offset))*flipped_kernel.at<float>(m,n);

		}

	   convolved_image.at<float>(j,i)  = value;
	   
	   value = 0;	 
       }

  Mat depadded_image = image_depadding(convolved_image, offset);
  
  return depadded_image;
}

void adaptive_threshold(const Mat& input_image, Mat& thresholded_image, float c, bool inverse=false)
{
   thresholded_image = input_image.clone();
   int image_type = input_image.type();

   if(image_type != CV_8UC1)
   	thresholded_image.convertTo(thresholded_image,CV_8UC1);

   Mat blurred_image;

   GaussianBlur( thresholded_image,blurred_image, Size( 5, 5 ), 1.4, 1.4);

   if(blurred_image.type() != CV_8UC1)
   	blurred_image.convertTo(blurred_image,image_type);

   float threshold;

   for(int j = 1; j < input_image.rows-1; j++)
	for(int i =1; i < input_image.cols-1; i++)
	{
		if(image_type == CV_8UC1)
		{
			threshold =(int) saturate_cast<uchar>(blurred_image.at<uchar>(j,i) - c);
	
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
			threshold = (blurred_image.at<float>(j,i) - c);

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

}



Mat harris_corner_detection(const Mat& input_image)
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
   Mat corners= Mat::zeros(horizontal_image.rows, horizontal_image.cols, CV_8UC1 );

   float k = 0.04;

   for(int j = 0; j < horizontal_image.rows ; j++)
      for(int i = 0; i < horizontal_image.cols ; i++)
      {
	   //R = Det(H) - k(Trace(H))^2;
   
	   harris_response.at<float>(j,i) = horizontal_image_squared.at<float>(j,i)*vertical_image_squared.at<float>(j,i) -pow(horizontal_vertical_image.at<float>(j,i),2) 
					    - k*(pow(horizontal_image_squared.at<float>(j,i)+vertical_image_squared.at<float>(j,i),2));
		
      }

   Mat corners2 = Mat::zeros(horizontal_image.rows, horizontal_image.cols, CV_8UC1 );
   
   int size = 5;

   int offset = (size+1)/2 - 1;

    normalize(harris_response, harris_response, 1,0,NORM_MINMAX);
    adaptive_threshold(harris_response, corners, -0.3);

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

    

    for (std::vector<IndexT>::iterator it = corner_indices.begin() ; it != corner_indices.end(); ++it)
    {
	cout<<"index row = "<<it->row<<" , col="<<it->col<<endl;
	corners2.at<uchar>(it->row,it->col) = 255;
	
    }	


   Mat image_with_corners = input_image.clone();

   int border_offset = 3;

    for( int j = border_offset; j <  corners2.rows-border_offset ; j++ )
     { for( int i = border_offset; i <  corners2.cols-border_offset; i++ )
	  {
	    if( (int)  corners2.at<uchar>(j,i) == 255 )
	      {
	       circle(  image_with_corners, Point( i, j ), 10,  Scalar(127), 2, 8, 0 );
	      }
	  }
     }

   return image_with_corners; 
}


int main(int argc, char** argv )
{
    Mat grayscale_image1, grayscale_image2;

    Mat image1 = imread( "../images/house.jpg", 1 );
    Mat image2 = imread( "../images/checkerboard.jpg", 1 );

    if ( !image1.data || !image2.data )
     {
        printf("No image data \n");
        return -1;
     }

    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
    cvtColor( image2, grayscale_image2, CV_BGR2GRAY );

    Mat corners1 = harris_corner_detection(grayscale_image1);

    Mat corners2 = harris_corner_detection(grayscale_image2);

    namedWindow("Input Image 1", WINDOW_AUTOSIZE);
    imshow("Input Image 1", image1); 

    namedWindow("Corner 1", WINDOW_AUTOSIZE);
    imwrite( "../results/harris_corner_detection/corners_1.jpg", corners1);
    imshow("Corner 1", corners1); 

    namedWindow("Input Image 2", WINDOW_AUTOSIZE);
    imshow("Input Image 2", image2); 

    namedWindow("Corner 2", WINDOW_AUTOSIZE);
    imwrite( "../results/harris_corner_detection/corners_2.jpg", corners2);
    imshow("Corner 2", corners2); 
	
    waitKey(0);

    return 0;
}

