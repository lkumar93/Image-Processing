#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

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

Mat morphological_filter(const Mat& input_image, int kernel_size,std::string type )
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

	   if(type == "erosion")
		filtered_image.at<uchar>(j,i)  = neighborhood[0]; 
	   
	   else if(type == "dilation")
		filtered_image.at<uchar>(j,i)  = neighborhood[k-1]; 
       }

  Mat depadded_image = image_depadding(filtered_image, offset);
  
  return depadded_image;

}

int main(int argc, char** argv )
{
    Mat image1, grayscale_image1, eroded_image1, dilated_image1,image2, grayscale_image2, eroded_image2, dilated_image2;

    image1 = imread( "../images/letter_a.jpg", 1 );
    image2 = imread( "../images/fingerprint.png", 1 );

    if ( !image1.data || !image2.data )
    {
        printf("No image data \n");
        return -1;
    }

    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
    eroded_image1 = morphological_filter(grayscale_image1,3,"erosion");
    dilated_image1 = morphological_filter(grayscale_image1,3,"dilation");

    cvtColor( image2, grayscale_image2, CV_BGR2GRAY );
    eroded_image2 = morphological_filter(grayscale_image2,3,"erosion");
    dilated_image2 = morphological_filter(grayscale_image2,3,"dilation");

    namedWindow("Original Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/morphological_filter/grayscale_image1.jpg", grayscale_image1 );
    imshow("Original Image 1", grayscale_image1);

    namedWindow("Eroded Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/morphological_filter/eroded_image1.jpg", eroded_image1 );
    imshow("Eroded Image 1", eroded_image1 );

    namedWindow("Dilated Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/morphological_filter/dilated_image1.jpg", dilated_image1 );
    imshow("Dilated Image 1", dilated_image1);

    namedWindow("Original Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/morphological_filter/grayscale_image2.jpg", grayscale_image2 );
    imshow("Original Image 2", grayscale_image2);

    namedWindow("Eroded Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/morphological_filter/eroded_image2.jpg", eroded_image2 );
    imshow("Eroded Image 2", eroded_image2 );

    namedWindow("Dilated Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/morphological_filter/dilated_image2.jpg", dilated_image2 );
    imshow("Dilated Image 2", dilated_image2 );

    waitKey(0);

    return 0;
}

