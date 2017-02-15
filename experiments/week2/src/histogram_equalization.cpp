#include <stdio.h>
#include <opencv2/opencv.hpp>

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

void compute_histogram(const Mat& input_image, int histogram[])
{

    for (int i = 0 ; i <256 ; i++)
	histogram[i] = 0;
  
    // Store the frequency of intensities
    for(int j = 0; j < input_image.rows; j++)
        for(int i = 0; i < input_image.cols; i++)
        {
	    histogram[input_image.at<uchar>(j,i)]++;
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

Mat equalize_image(const Mat& input_image, int histogram[])
{

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


int main(int argc, char** argv )
{
    Mat rgb_image, grayscale_image, equalized_image;
    rgb_image = imread( "../images/trees.jpg", 1 );

    if ( !rgb_image.data )
    {
        printf("No image data \n");
        return -1;
    }

    int histogram[256], equalized_histogram[256];

    grayscale_image = convert_to_grayscale(rgb_image);

    compute_histogram(grayscale_image, histogram);

    equalized_image = equalize_image(grayscale_image, histogram);
 
    compute_histogram(equalized_image, equalized_histogram);

    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imwrite( "../results/histogram_equalization/original_image.jpg", rgb_image );
    imshow("Original Image", grayscale_image);

    display_histogram(histogram, "original_histogram");

    namedWindow("Equalized Image", WINDOW_AUTOSIZE);
    imwrite( "../results/histogram_equalization/equalized_image.jpg", equalized_image );
    imshow("Equalized Image", equalized_image);

    display_histogram(equalized_histogram, "equalized_histogram");
    
    waitKey(0);

    return 0;
}

