#include <iostream>
#include <opencv2/opencv.hpp>

#define GAUSSIAN_FILTER 0
#define MEAN_FILTER 1
#define MEDIAN_FILTER 2

using namespace cv;
using namespace std;

void threshold_image(const Mat& input_image, Mat& thresholded_image, float threshold, bool inverse=false, bool adaptive=false, int filter_type = GAUSSIAN_FILTER, int size_k = 11)
{
   thresholded_image = input_image.clone();
   int image_type = input_image.type();
   thresholded_image.convertTo(thresholded_image,CV_8UC1);
   Mat blurred_image;

   if(filter_type == MEAN_FILTER)
	blur(thresholded_image, blurred_image, Size(size_k,size_k), Point(-1,-1));

   else if(filter_type == GAUSSIAN_FILTER)
 	GaussianBlur( thresholded_image,blurred_image, Size(size_k,size_k ), 4, 4);

   else if(filter_type == MEDIAN_FILTER)
 	medianBlur( thresholded_image,blurred_image, size_k);

   if(blurred_image.type() != CV_8UC1)
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

}

int main(int argc, char** argv )
{
    Mat grayscale_image1;

    Mat image1 = imread( "../images/article.png", 1 );

  if ( !image1.data)
    {
        printf("No image data \n");
        return -1;
    }

    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );

    GaussianBlur( grayscale_image1,grayscale_image1, Size( 7, 7), 3, 3);


    Mat mean_thresholded_image, gaussian_thresholded_image, median_thresholded_image, global_thresholded_image;

    int kernel_size = 11;
    int c = 1;

    threshold_image(grayscale_image1, mean_thresholded_image, c, false, true, MEAN_FILTER, kernel_size);
    threshold_image(grayscale_image1, gaussian_thresholded_image, c, false, true, GAUSSIAN_FILTER, kernel_size);
    threshold_image(grayscale_image1, median_thresholded_image, c, false, true, MEDIAN_FILTER, kernel_size);
    threshold_image(grayscale_image1, global_thresholded_image, 125, false, false);

    namedWindow("Input Image 1", WINDOW_AUTOSIZE);
    imshow("Input Image 1", grayscale_image1); 

    namedWindow("Mean Thresholded Image", WINDOW_AUTOSIZE);
    imwrite( "../results/adaptive_threshold/mean_thresholded_image.jpg", mean_thresholded_image);
    imshow("Mean Thresholded Image", mean_thresholded_image); 

    namedWindow("Gaussian Thresholded Image", WINDOW_AUTOSIZE);
    imwrite( "../results/adaptive_threshold/gaussian_thresholded_image.jpg", gaussian_thresholded_image);
    imshow("Gaussian Thresholded Image", gaussian_thresholded_image); 

    namedWindow("Median Thresholded Image", WINDOW_AUTOSIZE);
    imwrite( "../results/adaptive_threshold/median_thresholded_image.jpg", median_thresholded_image);
    imshow("Median Thresholded Image", median_thresholded_image); 

    namedWindow("Global Thresholded Image", WINDOW_AUTOSIZE);
    imwrite( "../results/adaptive_threshold/global_thresholded_image.jpg", global_thresholded_image);
    imshow("Global Thresholded Image", global_thresholded_image); 

    waitKey(0);

    return 0;
}

