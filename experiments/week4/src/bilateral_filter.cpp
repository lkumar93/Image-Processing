#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

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

Mat bilateral_convolve(const Mat& input_image, const Mat& gaussian_kernel, float sigma_r)
{
  
   int kernel_size = gaussian_kernel.rows;

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

  Mat convolved_image = Mat(padded_image.rows, padded_image.cols, CV_8UC1, 0.0);		

  float value, range_weight, weight, cumulative_weight;
  int pixel_intensity, neighboring_pixel_intensity ;

  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
	   
           int pixel_intensity = padded_image.at<uchar>(j,i);
 	   cumulative_weight = 0.0;
	   value = 0.0;
	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{
		    neighboring_pixel_intensity = padded_image.at<uchar>(j+m-offset,i+n-offset);
		    range_weight = (float) exp(-1.0* ( pow(pixel_intensity - neighboring_pixel_intensity,2) ) / ( 2.0 * pow(sigma_r,2) ) );
		    weight = gaussian_kernel.at<float>(m,n) * range_weight ;
		    value += neighboring_pixel_intensity*weight;
		    cumulative_weight += weight ;
		    //std::cout<<flipped_kernel.at<double>(m,n)<<std::endl;
		}

	   // Normalize the value
	   convolved_image.at<uchar>(j,i)  = (int) (value/cumulative_weight);	 
       }

  Mat depadded_image = image_depadding(convolved_image, offset);
  
  return depadded_image;
}

Mat bilateral_filter(const Mat& input_image, int kernel_size, float sigma_g, float sigma_r)
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

// Create the Gaussian Kernel
   for(int j = 0; j < kernel_size ; j++)
       for(int i = 0; i < kernel_size; i++)
       {
	    kernel.at<float>(j,i) = (float) exp(-1.0* ( pow(i+1-(k+1),2) + pow(j+1-(k+1),2) ) / ( 2.0 * pow(sigma_g,2) ) ) ; // ( 1.0 /( 2.0*PI*pow(sigma_g,2) ) ) *
       }

   Mat filtered_image = bilateral_convolve(input_image, kernel, sigma_r);

   return filtered_image;
}

int main(int argc, char** argv )
{
    Mat image1,image2, grayscale_image1,grayscale_image2, bilaterally_filtered_image1,bilaterally_filtered_image2, gaussian_filtered_image1,gaussian_filtered_image2;

    image1 = imread( "../images/bike.jpg", 1 );
    image2 = imread( "../images/tajmahal.jpg", 1 );

    if ( !image1.data || !image2.data )
    {
        printf("No image data \n");
        return -1;
    }

    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
    GaussianBlur(grayscale_image1, gaussian_filtered_image1, Size(7,7),0,0);
    bilaterally_filtered_image1 = bilateral_filter(grayscale_image1,7,15,10);

    cvtColor( image2, grayscale_image2, CV_BGR2GRAY );
    GaussianBlur(grayscale_image2, gaussian_filtered_image2, Size(7,7),0,0);
    bilaterally_filtered_image2 = bilateral_filter(grayscale_image2,7,15,10);


    namedWindow("Original Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/bilateral_filter/grayscale_image1.jpg", grayscale_image1 );
    imshow("Original Image 1", grayscale_image1);

    namedWindow("Bilaterally Filtered Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/bilateral_filter/bilaterally_filtered_image1.jpg", bilaterally_filtered_image1 );
    imshow("Bilaterally Filtered Image 1", bilaterally_filtered_image1 );

    namedWindow("Gaussian Filtered Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/bilateral_filter/gaussian_filtered_image1.jpg", gaussian_filtered_image1);
    imshow("Gaussian Filtered Image 1", gaussian_filtered_image1);


    namedWindow("Original Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/bilateral_filter/grayscale_image2.jpg", grayscale_image2 );
    imshow("Original Image 2", grayscale_image2);

    namedWindow("Bilaterally Filtered Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/bilateral_filter/bilaterally_filtered_image2.jpg", bilaterally_filtered_image2 );
    imshow("Bilaterally Filtered Image 2", bilaterally_filtered_image2 );

    namedWindow("Gaussian Filtered Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/bilateral_filter/gaussian_filtered_image2.jpg", gaussian_filtered_image2 );
    imshow("Gaussian Filtered Image 2", gaussian_filtered_image2 );



    waitKey(0);

    return 0;
}

