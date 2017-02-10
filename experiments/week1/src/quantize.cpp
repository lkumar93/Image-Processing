#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;


Mat quantize(const Mat& input_image,int number_of_bits)
{

    Mat quantized_image = input_image.clone();

    uchar masked_bit = 0xFF;

    masked_bit = masked_bit << (8 - number_of_bits);

    for(int j = 0; j < quantized_image.rows; j++)
        for(int i = 0; i < quantized_image.cols; i++)
        {
            Vec3b rgb_value = quantized_image.at<Vec3b>(j, i);
            rgb_value[0] = rgb_value[0] & masked_bit;
            rgb_value[1] = rgb_value[1] & masked_bit;
            rgb_value[2] = rgb_value[2] & masked_bit;
            quantized_image.at<Vec3b>(j, i) = rgb_value;
        }

    return quantized_image;

}

float mean_squared_error(const Mat& image_1, const Mat& image_2)
{

   int m = image_1.rows;
   int n = image_1.cols;

   float mse = 0.0;

    for(int j = 0; j < m ; j++)
        for(int i = 0; i < n; i++)
        {
            Vec3b rgb_value_1 = image_1.at<Vec3b>(j, i);
	    Vec3b rgb_value_2 = image_2.at<Vec3b>(j, i);

	    mse += abs(rgb_value_1[0]-rgb_value_2[0])^2 + abs(rgb_value_1[1]-rgb_value_2[1])^2 + abs(rgb_value_1[2]-rgb_value_2[2])^2 ;
        }

    return mse/(m*n*3);
}

int main(int argc, char** argv )
{
    Mat image, image_4_bit, image_2_bit, image_1_bit;
    image = imread( "../images/avengers.jpg", 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    image_4_bit = quantize(image,4);
    image_2_bit = quantize(image,2);
    image_1_bit = quantize(image,1);

    float mse_4_bit, mse_2_bit, mse_1_bit;

    mse_4_bit = mean_squared_error(image,image_4_bit);
    mse_2_bit = mean_squared_error(image,image_2_bit);
    mse_1_bit = mean_squared_error(image,image_1_bit);

    std::ostringstream str_4_bit, str_2_bit,str_1_bit;

    str_4_bit << "MSE of 4 Bit Image = " << mse_4_bit;
    str_2_bit << "MSE of 2 Bit Image = " << mse_2_bit;
    str_1_bit << "MSE of 1 Bit Image = " << mse_1_bit;

    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imwrite( "../results/quantization/original_image.jpg", image );
    imshow("Original Image", image);

    namedWindow("4 bit Image", WINDOW_AUTOSIZE);

    putText(image_4_bit, str_4_bit.str(), cvPoint(30,30), 
    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

    imwrite( "../results/quantization/4_bit_image.jpg", image_4_bit);
    imshow("4 bit Image", image_4_bit);

    namedWindow("2 bit Image", WINDOW_AUTOSIZE);

    putText(image_2_bit, str_2_bit.str(), cvPoint(30,30), 
    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


    imwrite( "../results/quantization/2_bit_image.jpg", image_2_bit);
    imshow("2 bit Image", image_2_bit);

    namedWindow("1 bit Image", WINDOW_AUTOSIZE);

    putText(image_1_bit, str_1_bit.str(), cvPoint(30,30), 
    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


    imwrite( "../results/quantization/1_bit_image.jpg", image_1_bit);
    imshow("1 bit Image", image_1_bit);

    waitKey(0);

    return 0;
}

