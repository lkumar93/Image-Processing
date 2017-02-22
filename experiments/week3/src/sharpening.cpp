#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159
#define R_WEIGHT 0.2989
#define G_WEIGHT 0.5870
#define B_WEIGHT 0.1140

using namespace cv;

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

Mat sharpen(const Mat& input_image, int kernel_size, float alpha)
{
  Mat low_pass_filtered_image, high_pass_filtered_image, sharpened_image;
  low_pass_filtered_image = input_image.clone();

  //Low pass filter the image using Gaussian
  blur( input_image, low_pass_filtered_image, Size( kernel_size, kernel_size ));

  high_pass_filtered_image = input_image.clone();
  sharpened_image = input_image.clone();  

  for(int j = 0; j < input_image.rows; j++)
	for(int i = 0; i < input_image.cols; i++)
	{

	    //High pass filter the image by subtracting input_image from low pass filtered image and saturate the output
	    high_pass_filtered_image.at<uchar>(j,i) = saturate_cast<uchar>(input_image.at<uchar>(j,i) - low_pass_filtered_image.at<uchar>(j,i));

	    //Sharpen the image by doing weighted addition of the high pass filtered image to the input image and saturate the output
	    sharpened_image.at<uchar>(j,i) =  saturate_cast<uchar>(input_image.at<uchar>(j,i) + alpha*high_pass_filtered_image.at<uchar>(j,i));
		
	}

  namedWindow("HPF Image", WINDOW_AUTOSIZE);
  imwrite( "../results/sharpening/high_pass_filtered_image.jpg", high_pass_filtered_image);
  imshow("HPF Image", high_pass_filtered_image);

  namedWindow("LPF Image", WINDOW_AUTOSIZE);
  imwrite( "../results/sharpening/low_pass_filtered_image.jpg", low_pass_filtered_image);
  imshow("LPF Image", low_pass_filtered_image);
  
  return sharpened_image;

}

int main(int argc, char** argv )
{
    Mat image, grayscale_image, sharpened_image, sharpened_image2, sharpened_image3;

    image = imread( "../images/moon.png", 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    grayscale_image = convert_to_grayscale(image);
    sharpened_image = sharpen(grayscale_image,9,0.5);

    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imwrite( "../results/sharpening/grayscale_image.jpg", grayscale_image );
    imshow("Original Image", grayscale_image);

    namedWindow("Sharpened Image", WINDOW_AUTOSIZE);
    imwrite( "../results/sharpening/sharpened_image.jpg", sharpened_image );
    imshow("Sharpened Image", sharpened_image);

    waitKey(0);

    return 0;
}

