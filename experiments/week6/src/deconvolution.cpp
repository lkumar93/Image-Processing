#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;

#define STD_DEV 50

#define KERNEL_SIZE 17

#define SIGMA 3

#define PI 3.14159

#define THRESHOLD 0.2

Mat image_padding(const Mat& input_image)
{
   Mat padded_image ;
   int rows = getOptimalDFTSize(input_image.rows);
   int cols = getOptimalDFTSize(input_image.cols);
   copyMakeBorder(input_image, padded_image, 0, rows - input_image.rows, 0, cols - input_image.cols, BORDER_CONSTANT, Scalar::all(0));
   return padded_image;

}

Mat add_noise(const Mat& input_image, int std_dev)
{
	Mat noisy_image(input_image.rows,input_image.cols,CV_8U);
	Mat noise(input_image.rows,input_image.cols,CV_32F);
        randn(noise,Scalar::all(0),Scalar::all(STD_DEV));
	noise.convertTo(noisy_image,CV_8U);
	noisy_image += input_image.clone();
	return noisy_image;
}


Mat pad_kernel(cv::Size size,const Mat& kernel, int kernel_size)
{
 
  Mat padded_kernel = Mat::zeros(size, CV_32F);

   for(int j = 0; j < kernel_size ; j++)
       for(int i = 0; i < kernel_size; i++)
       {
	
		padded_kernel.at<float>(j,i) = kernel.at<float>(j,i);

	}

   return padded_kernel;

}

Mat get_gaussian_blur_kernel(int kernel_size, float sigma)
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

   float value = 0.0;

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

   return kernel;
}

Mat get_motion_blur_kernel(int kernel_size)
{
       Mat kernel = Mat::zeros(kernel_size, kernel_size, CV_32F);

       int j = kernel_size/2;

       for(int i = 0; i < kernel_size; i++)
       {
	    kernel.at<float>(j,i) = 1.0/kernel_size ;
       }

       return kernel;
}

Mat fourier_transform(const Mat& padded_image)
{
	Mat image;
	padded_image.convertTo(image,CV_32F);
	//Real part and imaginary part
	Mat images[] = {Mat_<float>(image), Mat::zeros(image.size(), CV_32F)};
	Mat complex_image;
	merge(images, 2, complex_image);         
	dft(complex_image,complex_image);
	return complex_image;

}


Mat power_spectrum(const Mat& input_image)
{
	Mat complex_image = fourier_transform(input_image);
	Mat images[2],image_magnitude,spectrum_image;
	split(complex_image, images);
	magnitude(images[0],images[1],image_magnitude);
	multiply(image_magnitude,image_magnitude,spectrum_image);
	return spectrum_image;
}


Mat wiener_filter(const Mat& noisy_image, const Mat& signal_spectrum, Mat kernel, int kernel_size)
{
   Mat noise(noisy_image.rows,noisy_image.cols,CV_8U);
   randn(noise,Scalar::all(0),Scalar::all(STD_DEV));
  
   Scalar mean_input_image = mean(noisy_image);  

   Mat noise_spectrum = power_spectrum(noise);

   Mat images[2], kernel_images[2];

   Mat complex_image = fourier_transform(noisy_image);

   split(complex_image,images);
 
   Mat padded_kernel = pad_kernel(noisy_image.size(),kernel,kernel_size);

   Mat kernel_spectrum = power_spectrum(padded_kernel);

   Mat kernel_spectrum_squared;

   Mat kernel_complex_image = fourier_transform(padded_kernel);

   split(kernel_complex_image,kernel_images);

   multiply(kernel_spectrum,kernel_spectrum,kernel_spectrum_squared);
   
   Mat inv_snr = noise_spectrum/signal_spectrum;

   Mat weight = Mat::zeros(noisy_image.size(), CV_32F) ; 

   for(int j = 0; j < noisy_image.rows ; j++)
       for(int i = 0; i < noisy_image.cols; i++)
       {
	    if( kernel_spectrum.at<float>(j,i) > THRESHOLD)
	    	weight.at<float>(j,i) = ((kernel_spectrum_squared.at<float>(j,i))/(kernel_spectrum_squared.at<float>(j,i) + inv_snr.at<float>(j,i)))/kernel_spectrum.at<float>(j,i) ;
	
	    else
	    	weight.at<float>(j,i) = 0.0;
       }

   multiply(images[0],weight,images[0]);
   multiply(images[1],weight,images[1]);

   merge(images,2,complex_image);

   idft(complex_image,complex_image);
  
   split(complex_image,images);

   Scalar mean_restored_image = mean(images[0]);

   double scale_factor = mean_input_image.val[0]/mean_restored_image.val[0];

   multiply(images[0],scale_factor,images[0]);

   Mat normalized_image ;

   images[0].convertTo(normalized_image,CV_8UC1);

   return normalized_image;
}


Mat inverse_filter(const Mat& noisy_image, Mat kernel, int kernel_size, bool pseudo_inverse = false)
{
   Mat noise(noisy_image.rows,noisy_image.cols,CV_8U);
   randn(noise,Scalar::all(0),Scalar::all(STD_DEV));
  
   Scalar mean_input_image = mean(noisy_image);  

   Mat images[2], kernel_images[2];

   Mat complex_image = fourier_transform(noisy_image);

   split(complex_image,images);
 
   Mat padded_kernel = pad_kernel(noisy_image.size(),kernel,kernel_size);

   Mat kernel_spectrum = power_spectrum(padded_kernel);

   Mat kernel_spectrum_squared;

   Mat kernel_complex_image = fourier_transform(padded_kernel);

   split(kernel_complex_image,kernel_images);

   multiply(kernel_spectrum,kernel_spectrum,kernel_spectrum_squared);

   Mat weight = Mat::zeros(noisy_image.size(), CV_32F) ; 

   for(int j = 0; j < noisy_image.rows ; j++)
       for(int i = 0; i < noisy_image.cols; i++)
       {
	    if (pseudo_inverse)
	    {
		    if( kernel_spectrum.at<float>(j,i) > THRESHOLD)
		    	weight.at<float>(j,i) = (1/kernel_spectrum.at<float>(j,i)) ;
	
		    else
		    	weight.at<float>(j,i) = 0.0;
	    }

	    else
	    {
		    weight.at<float>(j,i) = (1/kernel_spectrum.at<float>(j,i)) ;

	    }

       }

   multiply(images[0],weight,images[0]);
   multiply(images[1],weight,images[1]);


   merge(images,2,complex_image);

   idft(complex_image,complex_image);
  
   split(complex_image,images);

   Scalar mean_restored_image = mean(images[0]);

   double scale_factor = mean_input_image.val[0]/mean_restored_image.val[0];

   multiply(images[0],scale_factor,images[0]);

   Mat normalized_image ;

   images[0].convertTo(normalized_image,CV_8UC1);

   return normalized_image;
}

int main(int argc, char** argv )
{
    Mat input_image,sample_image,resized_sample_image,padded_image,noisy_gaussian_blurred_image,noisy_motion_blurred_image,gaussian_blurred_image,motion_blurred_image;

    input_image = imread( "../images/horse.jpg", CV_LOAD_IMAGE_GRAYSCALE ) ;

    sample_image = imread( "../images/lena.jpg", CV_LOAD_IMAGE_GRAYSCALE ) ;

    if (!input_image.data)
    {
        printf("No image data \n");
        return -1;
    }

    //GaussianBlur(input_image,input_image,Size(3,3),1.5,1.5);

    padded_image = image_padding(input_image);

    Mat motion_blur_kernel = get_motion_blur_kernel(KERNEL_SIZE);

    Mat gaussian_blur_kernel = get_gaussian_blur_kernel(KERNEL_SIZE, SIGMA);

    filter2D(padded_image,motion_blurred_image,-1,motion_blur_kernel, Point(-1,-1),0,BORDER_DEFAULT);

    filter2D(padded_image,gaussian_blurred_image,-1,gaussian_blur_kernel, Point(-1,-1),0,BORDER_DEFAULT);

    //GaussianBlur(padded_image,gaussian_blurred_image,Size(KERNEL_SIZE,KERNEL_SIZE),SIGMA,SIGMA);

    noisy_gaussian_blurred_image =add_noise(gaussian_blurred_image,STD_DEV);

    noisy_motion_blurred_image =add_noise(motion_blurred_image,STD_DEV);
    
    resize(sample_image, resized_sample_image,padded_image.size());

    Mat sample_image_power_spectrum = power_spectrum(resized_sample_image);

    Mat restored_image2 = wiener_filter(noisy_motion_blurred_image,sample_image_power_spectrum,gaussian_blur_kernel,KERNEL_SIZE);	
    Mat restored_image1 = wiener_filter(noisy_gaussian_blurred_image,sample_image_power_spectrum,motion_blur_kernel,KERNEL_SIZE);

    Mat restored_image3 = inverse_filter(gaussian_blurred_image,gaussian_blur_kernel,KERNEL_SIZE);	
    Mat restored_image4 = inverse_filter(motion_blurred_image,motion_blur_kernel,KERNEL_SIZE);

    Mat restored_image5 = inverse_filter(gaussian_blurred_image,gaussian_blur_kernel,KERNEL_SIZE, true);	
    Mat restored_image6 = inverse_filter(motion_blurred_image,motion_blur_kernel,KERNEL_SIZE, true);

    Mat restored_image7 = inverse_filter(noisy_gaussian_blurred_image,gaussian_blur_kernel,KERNEL_SIZE, true);	
    Mat restored_image8 = inverse_filter(noisy_motion_blurred_image,motion_blur_kernel,KERNEL_SIZE, true);

    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imwrite( "../results/wiener_filter/original_image.jpg", padded_image);
    imshow("Original Image", padded_image);

    namedWindow("Noisy Gaussian Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/wiener_filter/noisy_gaussian_blurred_image.jpg", noisy_gaussian_blurred_image);
    imshow("Noisy Gaussian Blurred Image", noisy_gaussian_blurred_image);

    namedWindow("Noisy Motion Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/wiener_filter/noisy_motion_blurred_image.jpg", noisy_motion_blurred_image);
    imshow("Noisy Motion Blurred Image", noisy_motion_blurred_image);

    namedWindow("Restored Gaussian Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/wiener_filter/restored_gaussian_blurred_image.jpg", restored_image1);
    imshow("Restored Gaussian Blurred Image", restored_image1);

    namedWindow("Restored Motion Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/wiener_filter/restored_motion_blurred_image.jpg", restored_image2);
    imshow("Restored Motion Blurred Image", restored_image2);


    namedWindow("Gaussian Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/gaussian_blurred_image.jpg", gaussian_blurred_image);
    imshow("Gaussian Blurred Image", gaussian_blurred_image);

    namedWindow("Motion Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/motion_blurred_image.jpg", motion_blurred_image);
    imshow("Motion Blurred Image", motion_blurred_image);

    namedWindow("Inverse Motion Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/inverse_motion_blurred_image.jpg", restored_image4);
    imshow("Inverse Motion Blurred Image", restored_image4);

    namedWindow("Inverse Gaussian Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/inverse_gaussian_blurred_image.jpg", restored_image3);
    imshow("Inverse Gaussian Blurred Image", restored_image3);

    namedWindow("Pseudo Inverse Motion Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/pseudo_inverse_motion_blurred_image.jpg", restored_image6);
    imshow("Pseudo Inverse Motion Blurred Image", restored_image6);

    namedWindow("Pseudo Inverse Gaussian Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/pseudo_inverse_gaussian_blurred_image.jpg", restored_image5);
    imshow("Pseudo Inverse Gaussian Blurred Image", restored_image5);

    namedWindow("Pseudo Inverse Noisy Motion Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/pseudo_inverse_noisy_motion_blurred_image.jpg", restored_image8);
    imshow("Pseudo Inverse Noisy Motion Blurred Image", restored_image8);

    namedWindow("Pseudo Inverse Noisy Gaussian Blurred Image", WINDOW_AUTOSIZE);
    imwrite( "../results/inverse_filter/pseudo_inverse_noisy_gaussian_blurred_image.jpg", restored_image7);
    imshow("Pseudo Inverse Noisy Gaussian Blurred Image", restored_image7);
    waitKey(0);

    return 0;
}
