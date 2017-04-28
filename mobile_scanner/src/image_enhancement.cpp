//
// THIS FILE CONTAINS VARIOUS FUNCTIONS FOR ENHANCING IMAGES
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

#include "image_enhancement.h"
#include "image_filters.h"

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

Mat convert_to_grayscale(const Mat& input_image)
{

    Mat rgb_image = input_image.clone();

    if(rgb_image.type() != CV_8UC1)
  	rgb_image.convertTo(rgb_image, CV_8UC1);

    Mat grayscale_image = cv::Mat(rgb_image.rows, rgb_image.cols, CV_8UC1, cv::Scalar(0, 0, 0));

    for(int j = 0; j < grayscale_image.rows; j++)
        for(int i = 0; i < grayscale_image.cols; i++)
        {
            Vec3b rgb_value = rgb_image.at<Vec3b>(j, i);
	    grayscale_image.at<uchar>(j,i) = R_WEIGHT*rgb_value[0] + G_WEIGHT*rgb_value[1] + B_WEIGHT*rgb_value[2] ;
        }

    return grayscale_image;

}

void compute_histogram(const Mat& input_image, int histogram[])
{
    Mat image = input_image.clone();

    if(image.type() != CV_8UC1)
  	image.convertTo(image, CV_8UC1);

    for (int i = 0 ; i <256 ; i++)
	histogram[i] = 0;
  
    // Store the frequency of intensities
    for(int j = 0; j < image.rows; j++)
        for(int i = 0; i < image.cols; i++)
        {
	    histogram[image.at<uchar>(j,i)]++;
        }

}

void display_histogram(int histogram[], const char* name)
{
    int hist[256];

    for(int i = 0; i < 256; i++)
    {
        hist[i]=histogram[i];
    }

    // draw the histograms
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double) hist_w/256);
 
    Mat histogram_image(hist_h, hist_w, CV_8UC1, Scalar(255, 255, 255));
 
    // find the maximum intensity element from histogram
    int max = hist[0];
    for(int i = 1; i < 256; i++){
        if(max < hist[i]){
            max = hist[i];
        }
    }
 
    // normalize the histogram between 0 and histImage.rows
 
    for(int i = 0; i < 256; i++){
        hist[i] = ((double)hist[i]/max)*histogram_image.rows;
    }
 
 
    // draw the intensity line for histogram
    for(int i = 0; i < 256; i++)
    {
        line(histogram_image, Point(bin_w*(i), hist_h),
                              Point(bin_w*(i), hist_h - hist[i]),
             Scalar(0,0,0), 1, 8, 0);
    }
 
    // display histogram
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, histogram_image);

    std::ostringstream display_string ;

    display_string<<"../results/histogram_equalization/"<< name <<".jpg";

    imwrite( display_string.str(), histogram_image );
}

void compute_cumulative_histogram(int histogram[], int cumulative_histogram[])
{
    cumulative_histogram[0] = histogram[0];
  
    for(int j = 1; j < 256 ; j++)
    {
	cumulative_histogram[j] = histogram[j]+cumulative_histogram[j-1];
    }
}

int scale_histogram(int cumulative_histogram[],int scaled_histogram[], float scaling_factor)
{
  
    for(int j = 0; j < 256 ; j++)
    {
	scaled_histogram[j] = cvRound(cumulative_histogram[j]*scaling_factor);
    }

}

Mat equalize_image(const Mat& input_image)
{
   int histogram[256];

   compute_histogram(input_image, histogram);

   int cumulative_histogram[256], scaled_histogram[256];

   int image_size = input_image.rows*input_image.cols;

   // Compute alpha (scaling factor) based on total number of pixels and maximum intensity   
   float scaling_factor = 255.0/image_size;


   // Compute probability of each intensity by normalizing the histogram
   double intensity_probability[256];
   
   for(int i = 0; i < 256 ; i++)
   {
      intensity_probability[i] =  histogram[i]/image_size;
   }


   // Compute the cumulative histogram by adding all values of lower intensities
   compute_cumulative_histogram(histogram, cumulative_histogram);

   // Scaled the cumulative histogram by the scaling factor
   scale_histogram(cumulative_histogram, scaled_histogram, scaling_factor);

   Mat equalized_image = input_image.clone();

   // Equalize the image by looking at the value from scaled histogram at the intensity 
   // level at the corresponding pixel

   for(int j = 0; j < equalized_image.rows; j++)
       for(int i = 0; i < equalized_image.cols; i++)
       {
	    equalized_image.at<uchar>(j,i) = saturate_cast<uchar>(scaled_histogram[equalized_image.at<uchar>(j,i)]);
       }   

  
   return equalized_image;

}

Mat log_transformation(const Mat& input_image, int transformation_constant)
{

   Mat log_image = input_image.clone();

  if(log_image.type() != CV_8UC1)
  	log_image.convertTo(log_image, CV_8UC1);

   for(int j = 0; j < log_image.rows ; j++)
        for(int i = 0; i < log_image.cols; i++)
        {
	    log_image.at<uchar>(j,i) = transformation_constant*log(abs(log_image.at<uchar>(j,i))+1);
        }

    return log_image;
}

Mat inverse_transformation(const Mat& input_image)
{
  Mat inverse_image = input_image.clone();

  if(inverse_image.type() != CV_8UC1)
  	inverse_image.convertTo(inverse_image, CV_8UC1);

   for(int j = 0; j < inverse_image.rows ; j++)
        for(int i = 0; i < inverse_image.cols; i++)
        {
	    inverse_image.at<uchar>(j,i) = 255 - inverse_image.at<uchar>(j,i);
        }

    return inverse_image;
}

Mat gamma_correction(const Mat& input_image, int gamma)
{
  Mat gamma_corrected_image = input_image.clone();

  if(gamma_corrected_image.type() != CV_8UC1)
  	gamma_corrected_image.convertTo(gamma_corrected_image, CV_8UC1);

  for(int j = 0; j < gamma_corrected_image.rows ; j++)
       for(int i = 0; i < gamma_corrected_image.cols; i++)
       {
	    gamma_corrected_image.at<uchar>(j,i) = pow( gamma_corrected_image.at<uchar>(j,i) , gamma );
       }

  return gamma_corrected_image;
}

Mat sharpen(const Mat& input_image, int kernel_size, float alpha)
{
  Mat image, low_pass_filtered_image, high_pass_filtered_image, sharpened_image;
  image = input_image.clone();

  if(image.type()!=CV_8UC1)
	image.convertTo(image,CV_8UC1);  

  //Low pass filter the image using Mean Filter
  low_pass_filtered_image = mean_filter(image, kernel_size);

  high_pass_filtered_image = image;
  sharpened_image = image;  

  for(int j = 0; j < input_image.rows; j++)
	for(int i = 0; i < input_image.cols; i++)
	{

	    //High pass filter the image by subtracting input_image from low pass filtered image and saturate the output
	    high_pass_filtered_image.at<uchar>(j,i) = saturate_cast<uchar>(input_image.at<uchar>(j,i) - low_pass_filtered_image.at<uchar>(j,i));

	    //Sharpen the image by doing weighted addition of the high pass filtered image to the input image and saturate the output
	    sharpened_image.at<uchar>(j,i) =  saturate_cast<uchar>(input_image.at<uchar>(j,i) + alpha*high_pass_filtered_image.at<uchar>(j,i));
		
	}

  return sharpened_image;

}

void threshold_image(const Mat& input_image, Mat& thresholded_image, float threshold, bool inverse, bool adaptive)
{
   thresholded_image = input_image.clone();
   int image_type = input_image.type();
   thresholded_image.convertTo(thresholded_image,CV_8UC1);
   Mat blurred_image;

   if(image_type == CV_32F)
	blurred_image = gaussian_filter(thresholded_image,5,1.4);
   else
   	blurred_image = gaussian_filter(input_image,11,1.8);

   if(blurred_image.type() != image_type)
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

