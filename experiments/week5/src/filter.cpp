#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;

int main(int argc, char** argv )
{
    Mat image1, grayscale_image1, equalized_image1;
    Mat image2, grayscale_image2, equalized_image2, gaussian_filtered_image2, box_filtered_image2;
    Mat image3, grayscale_image3, median_filtered_image3;
    Mat image4, grayscale_image4, median_filtered_image4;
    Mat image5, grayscale_image5, mean_filtered_image5;

    image1 = imread( "../images/631Lab1-brightim.jpg", 1 );
    image2 = imread( "../images/631Lab1-darkim.jpg", 1 );
    image3 = imread( "../images/631Lab1-noisyLena.tif", 1 );

    //http://cnx.org/resources/4181ddf62e3a2047d36357f16180ce247b094532/plane_noise1.png
    image4 = imread( "../images/noisy_plane.png", 1 );

    //en.wikipedia.org/wiki/File:Phase_correlation.png
    image5 = imread( "../images/noisy_lion.png", 1 );


    if ( !image1.data || !image2.data || !image3.data || !image4.data || !image5.data)
    {
        printf("No image data \n");
        return -1;
    }

    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
    cvtColor( image2, grayscale_image2, CV_BGR2GRAY );
    cvtColor( image3, grayscale_image3, CV_BGR2GRAY );
    cvtColor( image4, grayscale_image4, CV_BGR2GRAY );
    cvtColor( image5, grayscale_image5, CV_BGR2GRAY );

    equalizeHist( grayscale_image1, equalized_image1 );
    equalizeHist( grayscale_image2, equalized_image2 );

    GaussianBlur(equalized_image2 ,gaussian_filtered_image2 ,Size(7,7), 1.4, 1.4, BORDER_DEFAULT);
    medianBlur(grayscale_image3 ,median_filtered_image3 , 7);
    medianBlur(grayscale_image4 ,median_filtered_image4 , 9);
     blur(grayscale_image5 ,mean_filtered_image5 , Size(7,7), Point(-1,-1));
  

    namedWindow("Equalized Image 1", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/equalized_image_1.jpg", equalized_image1);
    imshow("Equalized Image 1", equalized_image1);

    namedWindow("Equalized Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/equalized_image_2.jpg", equalized_image2);
    imshow("Equalized Image 2", equalized_image2);

    namedWindow("Gaussian Filtered Image 2", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/gaussian_filtered_image_2.jpg", gaussian_filtered_image2 );
    imshow("Gaussian Filtered Image 2", gaussian_filtered_image2);

    namedWindow("Median Filtered Image 3", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/median_filtered_image_3.jpg", median_filtered_image3 );
    imshow("Median Filtered Image 3", median_filtered_image3);

    namedWindow("Median Filtered Image 4", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/median_filtered_image_4.jpg", median_filtered_image4 );
    imshow("Median Filtered Image 4", median_filtered_image4);

    namedWindow("Mean Filtered Image 5", WINDOW_AUTOSIZE);
    imwrite( "../results/filter/mean_filtered_image_5.jpg", mean_filtered_image5 );
    imshow("Mean Filtered Image 5", mean_filtered_image5);

    waitKey(0);

    return 0;
}
