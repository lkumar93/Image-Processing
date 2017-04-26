//
// THIS FILE CONTAINS THE IMPLEMENTATION OF MORPHOLOGICAL FILTER

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

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

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



void merge(float A[ ] , int start, int mid, int end) {

   //stores the starting position of both parts in temporary variables.
    int p = start ,q = mid+1;

    float Arr[end-start+1];
    int k=0;

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

void merge_sort (float A[ ] , int start , int end ) {
    if( start < end ) {
       int mid = (start + end ) / 2 ;           // defines the current array in 2 parts .
       merge_sort (A, start , mid ) ;                 // sort the 1st part of array .
       merge_sort (A,mid+1 , end ) ;              // sort the 2nd part of array.

     // merge the both parts by comparing elements of both the parts.
        merge(A,start , mid , end );   
   }                    
}

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

Mat morphological_filter(const Mat& input_image, int kernel_size, bool max, bool ignore_center_pixel=false )
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

  Mat filtered_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);

  int size = kernel_size*kernel_size;
  
  float neighborhood[size];

  int k;
		

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
	   k = 0;	

	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{
		    int row = j+m-offset;
	  	    int col = i+n-offset;
		
		    if(ignore_center_pixel)
		    {		    
			    if(j!=row && i!=col)
			    {
				neighborhood[k] = padded_image.at<float>(row,col) ;
			        k++;
			    }
		     }
		    else
		     {
			neighborhood[k] = padded_image.at<float>(row,col) ;
			k++;

		     }

		}

	   if(ignore_center_pixel)
		merge_sort(neighborhood,0,size-2);
	   else
		merge_sort(neighborhood,0,size-1);
		

	   if(max)
	   {
		if(ignore_center_pixel)
			filtered_image.at<float>(j,i)  = neighborhood[k-2];
		else
			filtered_image.at<float>(j,i)  = neighborhood[k-1];
	   }	
	   else
		filtered_image.at<float>(j,i)  = neighborhood[0]; 

       }

  Mat depadded_image = image_depadding(filtered_image, offset);
  
  return depadded_image;

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
  float neighborhood[size];

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



///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

//int main(int argc, char** argv )
//{
//    Mat image1, grayscale_image1, eroded_image1, dilated_image1,image2, grayscale_image2, eroded_image2, dilated_image2;

//    image1 = imread( "../images/letter_a.jpg", 1 );
//    image2 = imread( "../images/fingerprint.png", 1 );

//    if ( !image1.data || !image2.data )
//    {
//        printf("No image data \n");
//        return -1;
//    }

//    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
//    eroded_image1 = morphological_filter(grayscale_image1,3,false);
//    dilated_image1 = morphological_filter(grayscale_image1,3,true);

//    cvtColor( image2, grayscale_image2, CV_BGR2GRAY );
//    eroded_image2 = morphological_filter(grayscale_image2,3,false);
//    dilated_image2 = morphological_filter(grayscale_image2,3,true);

//    namedWindow("Original Image 1", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/grayscale_image1.jpg", grayscale_image1 );
//    imshow("Original Image 1", grayscale_image1);

//    namedWindow("Eroded Image 1", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/eroded_image1.jpg", eroded_image1 );
//    imshow("Eroded Image 1", eroded_image1 );

//    namedWindow("Dilated Image 1", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/dilated_image1.jpg", dilated_image1 );
//    imshow("Dilated Image 1", dilated_image1);

//    namedWindow("Original Image 2", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/grayscale_image2.jpg", grayscale_image2 );
//    imshow("Original Image 2", grayscale_image2);

//    namedWindow("Eroded Image 2", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/eroded_image2.jpg", eroded_image2 );
//    imshow("Eroded Image 2", eroded_image2 );

//    namedWindow("Dilated Image 2", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/dilated_image2.jpg", dilated_image2 );
//    imshow("Dilated Image 2", dilated_image2 );

//    waitKey(0);

//    return 0;
//}

