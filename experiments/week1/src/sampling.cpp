#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    Mat image;
    image = imread( "../images/mickey.jpg", 1 );

    cv::Mat resized_image1 = cv::Mat(240, 320, CV_8UC3, cv::Scalar(0, 0, 0)) ;
    cv::Mat resized_image2 = cv::Mat(24, 32, CV_8UC3, cv::Scalar(0, 0, 0)) ;

    int rows = 600;
    int cols = 800;

    resize(image,resized_image1,resized_image1.size(),0,0);
    resize(image,resized_image2,resized_image2.size(),0,0);


    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    namedWindow("Original Image", WINDOW_NORMAL);
    resizeWindow("Original Image",rows,cols);
    imwrite( "../results/sampling/original_image.jpg", image );
    imshow("Original Image", image);


    namedWindow("320X240 Image", WINDOW_NORMAL );
    resizeWindow("320X240 Image",rows,cols);
    imwrite( "../results/sampling/320_x_240_image.jpg", resized_image1);
    imshow("320X240 Image", resized_image1);

    namedWindow("32X24 Image", WINDOW_NORMAL );
    resizeWindow("32X24 Image",rows,cols);
    imwrite( "../results/sampling/32_x_24_image.jpg", resized_image2 );
    imshow("32X24 Image", resized_image2);

    waitKey(0);

    return 0;
}

