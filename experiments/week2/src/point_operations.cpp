#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <math.h>

#define R_WEIGHT 0.2989
#define G_WEIGHT 0.5870
#define B_WEIGHT 0.1140
#define TRANSFORMATION_CONSTANT 25
#define GAMMA 0.95

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


Mat log_transformation(const Mat& input_image)
{

   Mat log_image = input_image.clone();

   for(int j = 0; j < log_image.rows ; j++)
        for(int i = 0; i < log_image.cols; i++)
        {
	    log_image.at<uchar>(j,i) = TRANSFORMATION_CONSTANT*log(log_image.at<uchar>(j,i)+1);
        }

    return log_image;
}

Mat inverse_transformation(const Mat& input_image)
{

   Mat inverse_image = input_image.clone();

   for(int j = 0; j < inverse_image.rows ; j++)
        for(int i = 0; i < inverse_image.cols; i++)
        {
	    inverse_image.at<uchar>(j,i) = 255 - inverse_image.at<uchar>(j,i);
        }

    return inverse_image;
}

Mat gamma_correction(const Mat& input_image)
{

   Mat gamma_corrected_image = input_image.clone();

   for(int j = 0; j < gamma_corrected_image.rows ; j++)
        for(int i = 0; i < gamma_corrected_image.cols; i++)
        {
	    gamma_corrected_image.at<uchar>(j,i) = pow( gamma_corrected_image.at<uchar>(j,i) , GAMMA );
        }

    return gamma_corrected_image;
}

int main(int argc, char** argv )
{
    Mat image, grayscale_image, log_image, inverse_image, gamma_corrected_image;

    image = imread( "../images/einstein.jpg", 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    grayscale_image = convert_to_grayscale(image);
    inverse_image = inverse_transformation(grayscale_image);
    log_image = log_transformation(grayscale_image);
    gamma_corrected_image = gamma_correction(grayscale_image);

    namedWindow("RGB Image", WINDOW_AUTOSIZE);
    imwrite( "../results/point_transformations/rgb_image.jpg", image );
    imshow("RGB Image", image);

    namedWindow("Grayscale Image", WINDOW_AUTOSIZE);
    imwrite( "../results/point_transformations/grayscale_image.jpg", grayscale_image );
    imshow("Grayscale Image", grayscale_image);

    namedWindow("Inverse Image", WINDOW_AUTOSIZE);
    imwrite( "../results/point_transformations/inverse_image.jpg", inverse_image );
    imshow("Inverse Image", inverse_image);

    namedWindow("Log Image", WINDOW_AUTOSIZE);
    imwrite( "../results/point_transformations/log_image.jpg", log_image );
    imshow("Log Image", log_image);

    namedWindow("Gamma Corrected Image", WINDOW_AUTOSIZE);
    imwrite( "../results/point_transformations/gamma_corrected_image.jpg", gamma_corrected_image );
    imshow("Gamma Corrected Image", gamma_corrected_image);

    waitKey(0);

    return 0;
}

