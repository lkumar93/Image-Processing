#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159
#define R_WEIGHT 0.2989
#define G_WEIGHT 0.5870
#define B_WEIGHT 0.1140

using namespace cv;

void swap(int & a1,int & a2)
{
	a1 = a1 + a2;
	a2 = a1 - a2;
	a1 = a1 - a2;
}

void merge(int A[ ] , int start, int mid, int end) {

   //stores the starting position of both parts in temporary variables.
    int p = start ,q = mid+1;

    int Arr[end-start+1] , k=0;

    for(int i = start ;i <= end ;i++) {
        if(p > mid)      //checks if first part comes to an end or not .
           Arr[ k++ ] = A[ q++] ;

       else if ( q > end)   //checks if second part comes to an end or not
           Arr[ k++ ] = A[ p++ ];

       else if( A[ p ] < A[ q ])     //checks which part has smaller element.
          Arr[ k++ ] = A[ p++ ];

       else
          Arr[ k++ ] = A[ q++];
   }
   for (int p=0 ; p< k ;p ++) {
     /* Now the real array has elements in sorted manner including both 
            parts.*/
       A[ start++ ] = Arr[ p ] ;                          
   }
}

void merge_sort (int A[ ] , int start , int end ) {
    if( start < end ) {
       int mid = (start + end ) / 2 ;           // defines the current array in 2 parts .
       merge_sort (A, start , mid ) ;                 // sort the 1st part of array .
       merge_sort (A,mid+1 , end ) ;              // sort the 2nd part of array.

     // merge the both parts by comparing elements of both the parts.
        merge(A,start , mid , end );   
   }                    
}


Mat convert_to_grayscale(const Mat& input_image)
{

    Mat rgb_image = input_image.clone();
    Mat grayscale_image = cv::Mat(rgb_image.rows, rgb_image.cols, CV_8UC1, cv::Scalar(0, 0, 0));

    for(int j = 0; j < grayscale_image.rows; j++)
        for(int i = 0; i < grayscale_image.cols; i++)
        {
            Vec3b rgb_value = rgb_image.at<Vec3b>(j, i);
	    grayscale_image.at<uchar>(j,i) = R_WEIGHT*rgb_value[0] + G_WEIGHT*rgb_value[1] + B_WEIGHT*rgb_value[2] ;
        }

    return grayscale_image;

}

Mat image_padding(const Mat& input_image, int offset)
{

   Mat padded_image = Mat(input_image.rows+2*offset, input_image.cols+2*offset, CV_8UC1, 0.0);

   for(int j = 0; j < input_image.rows ; j++)
        for(int i = 0; i < input_image.cols; i++)
        {
	    padded_image.at<uchar>(j+offset,i+offset) = input_image.at<uchar>(j,i);
        }

    return padded_image;

}

Mat image_depadding(const Mat& input_image, int offset)
{

   Mat depadded_image = Mat(input_image.rows-2*offset, input_image.cols-2*offset, CV_8UC1, 0.0);

   for(int j = 0; j < input_image.rows-2*offset ; j++)
        for(int i = 0; i < input_image.cols-2*offset; i++)
        {
	    depadded_image.at<uchar>(j,i) = input_image.at<uchar>(j+offset,i+offset);
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

		//std::cout<<"m = "<< m <<","<<kernel_size-m-1<<" ,n = " <<n<<","<<kernel_size-n-1<<" ,v="<<flipped_kernel.at<float>(m,n) <<std::endl;
	}

  Mat convolved_image = Mat(padded_image.rows, padded_image.cols, CV_8UC1, 0.0);
		

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
		   
	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{

		    value += (padded_image.at<uchar>(j+m-offset,i+n-offset))*flipped_kernel.at<float>(m,n);
		    //std::cout<<flipped_kernel.at<double>(m,n)<<std::endl;
		}

	   convolved_image.at<uchar>(j,i)  = (int)value;
	   value = 0;	 
       }

  Mat depadded_image = image_depadding(convolved_image, offset);
  
  return depadded_image;
}

Mat gaussian_filter(const Mat& input_image, int kernel_size, float sigma)
{

   Mat kernel = Mat(kernel_size, kernel_size, CV_32F, 0.0);
   int k;

   if(kernel_size % 2 != 0)
   {
	k = (kernel_size-1)/2;
   }
   else
   {
	k = (kernel_size)/2;
   }

   float value = 0.0,value2 = 0.0;

// Create the Gaussian Kernel
   for(int j = 0; j < kernel_size ; j++)
       for(int i = 0; i < kernel_size; i++)
       {
	    kernel.at<float>(j,i) = (float)( 1.0 /( 2.0*PI*pow(sigma,2) ) ) * exp(-1.0* ( pow(i+1-(k+1),2) + pow(j+1-(k+1),2) ) / ( 2.0 * pow(sigma,2) ) ) ;
	    value += kernel.at<float>(j,i);
       }

 // Normalize kernel , so that the sum of all elements in the kernel is 1
   for(int j = 0; j < kernel_size ; j++)
       for(int i = 0; i < kernel_size; i++)
       {
	    kernel.at<float>(j,i) = kernel.at<float>(j,i)/value ;
       }

   Mat filtered_image = convolve(input_image, kernel);

   return filtered_image;
}

Mat mean_filter(const Mat& input_image, int kernel_size)
{

   Mat kernel = Mat(kernel_size, kernel_size, CV_32F, 0.0);

   for(int j = 0; j < kernel_size ; j++)
       for(int i = 0; i < kernel_size; i++)
       {
	    kernel.at<float>(j,i) = ( 1.0 / pow(kernel_size,2)) ;
       }


   Mat filtered_image = convolve(input_image, kernel);  

   return filtered_image;
}


Mat median_filter(const Mat& input_image, int kernel_size)
{
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

  Mat filtered_image = Mat(padded_image.rows, padded_image.cols, CV_8UC1, 0.0);

  int size = kernel_size*kernel_size;
  int neighborhood[size];

  int k;

  int mid ;

   if(kernel_size % 2 != 0)
   { 
	mid = (size+1)/2-1;
   }
   else
   {
	mid = (size)/2-1;
   } 		

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
	   k = 0;	

	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{
		
		    neighborhood[k] = padded_image.at<uchar>(j+m-offset,i+n-offset) ;
		    k++;
		}

	    merge_sort(neighborhood,0,size-1);

            filtered_image.at<uchar>(j,i)  = neighborhood[mid]; 
       }

  Mat depadded_image = image_depadding(filtered_image, offset);
  
  return depadded_image;

}

int main(int argc, char** argv )
{
    Mat image, grayscale_image, gaussian_filtered_image, mean_filtered_image, median_filtered_image;

    image = imread( "../images/balloons.png", 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    grayscale_image = convert_to_grayscale(image);
    gaussian_filtered_image = gaussian_filter(grayscale_image,7,1.4);
    mean_filtered_image = mean_filter(grayscale_image,7);
    median_filtered_image = median_filter(grayscale_image,7);

    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/grayscale_image.jpg", grayscale_image );
    imshow("Original Image", grayscale_image);

    namedWindow("Gaussian Filtered Image", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/gaussian_filtered_image.jpg", gaussian_filtered_image );
    imshow("Gaussian Filtered Image", gaussian_filtered_image );

    namedWindow("Mean Filtered Image", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/mean_filtered_image.jpg", mean_filtered_image );
    imshow("Mean Filtered Image", mean_filtered_image);

    namedWindow("Median Filtered Image2", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/median_filtered_image.jpg", median_filtered_image );
    imshow("Median Filtered Image", median_filtered_image);


    waitKey(0);

    return 0;
}

