//
// THIS FILE CONTAINS THE IMPLEMENTATION OF VARIOUS IMAGE FILTERS

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

#include "image_filters.h"

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

void merge(float A[ ] , int start, int mid, int end) {

   //stores the starting position of both parts in temporary variables.
    int p = start ,q = mid+1;

    float Arr[end-start+1];
    int k=0;

    for(int i = start ;i <= end ;i++) {
        if(p > mid)      //checks if first part comes to an end or not .
           Arr[ k++ ] = A[ q++] ;

       else if ( q > end)   //checks if second part comes to an end or not
           Arr[ k++ ] = A[ p++ ];

       else if( A[ p ] < A[ q ])     //checks which part has smaller element.
          Arr[ k++ ] = A[ p++ ];

       else
          Arr[ k++ ] = A[ q++];
   }
   for (int p=0 ; p< k ;p ++) {
     /* Now the real array has elements in sorted manner including both 
            parts.*/
       A[ start++ ] = Arr[ p ] ;                          
   }
}

void merge_sort (float A[ ] , int start , int end ) {
    if( start < end ) {
       int mid = (start + end ) / 2 ;           // defines the current array in 2 parts .
       merge_sort (A, start , mid ) ;                 // sort the 1st part of array .
       merge_sort (A,mid+1 , end ) ;              // sort the 2nd part of array.

     // merge the both parts by comparing elements of both the parts.
        merge(A,start , mid , end );   
   }                    
}

Mat image_padding(const Mat& input_image, int offset)
{
   Mat padded_image = Mat(input_image.rows+2*offset, input_image.cols+2*offset, CV_32F, 0.0);

   Mat image = input_image.clone();

//   if(image.type() != CV_8UC1)
//  	image.convertTo(image, CV_8UC1);

   for(int j = 0; j < input_image.rows ; j++)
        for(int i = 0; i < input_image.cols; i++)
        {
	    if(image.type() == CV_8UC1)
	   	 padded_image.at<float>(j+offset,i+offset) = image.at<uchar>(j,i);

	    else if(image.type() == CV_32F)
	   	 padded_image.at<float>(j+offset,i+offset) = image.at<float>(j,i);
	
        }

    return padded_image;
}

Mat image_depadding(const Mat& input_image, int offset)
{
   Mat depadded_image = Mat(input_image.rows-2*offset, input_image.cols-2*offset, CV_32F, 0.0);

   for(int j = 0; j < input_image.rows-2*offset ; j++)
        for(int i = 0; i < input_image.cols-2*offset; i++)
        {
	    depadded_image.at<float>(j,i) = input_image.at<float>(j+offset,i+offset);
        }

    return depadded_image;
}

Mat convolve(const Mat& input_image, const Mat& kernel)
{
  
   int kernel_size = kernel.rows;

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

  Mat flipped_kernel = Mat(kernel.rows, kernel.cols, CV_32F, 0.0);

   for(int m = 0; m < kernel_size ; m++)
	for(int n = 0; n < kernel_size; n++)
	{
		flipped_kernel.at<float>(m,n) = kernel.at<float>(kernel_size-m-1,kernel_size-n-1);
	}

  Mat convolved_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);	

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
		  
	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{

		    value += (padded_image.at<float>(j+m-offset,i+n-offset))*flipped_kernel.at<float>(m,n);

		}

	   convolved_image.at<float>(j,i)  = value;
	   
	   value = 0;	 
       }

  Mat depadded_image = image_depadding(convolved_image, offset);
  
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

  Mat convolved_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);		

  float value, range_weight, weight, cumulative_weight;
  int pixel_intensity, neighboring_pixel_intensity ;

  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
	   
           int pixel_intensity = padded_image.at<float>(j,i);
 	   cumulative_weight = 0.0;
	   value = 0.0;
	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{
		    neighboring_pixel_intensity = padded_image.at<float>(j+m-offset,i+n-offset);
		    range_weight = (float) exp(-1.0* ( pow(pixel_intensity - neighboring_pixel_intensity,2) ) / ( 2.0 * pow(sigma_r,2) ) );
		    weight = gaussian_kernel.at<float>(m,n) * range_weight ;
		    value += neighboring_pixel_intensity*weight;
		    cumulative_weight += weight ;
		    //std::cout<<flipped_kernel.at<double>(m,n)<<std::endl;
		}

	   // Normalize the value
	   convolved_image.at<float>(j,i)  = (value/cumulative_weight);	 
       }

  Mat depadded_image = image_depadding(convolved_image, offset);
  
  return depadded_image;
}



Mat gaussian_filter(const Mat& input_image, int kernel_size, float sigma)
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

   float value = 0.0,value2 = 0.0;

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

   Mat filtered_image = convolve(input_image, kernel);

   return filtered_image;
}

Mat mean_filter(const Mat& input_image, int kernel_size)
{

   Mat kernel = Mat(kernel_size, kernel_size, CV_32F, 0.0);

   for(int j = 0; j < kernel_size ; j++)
       for(int i = 0; i < kernel_size; i++)
       {
	    kernel.at<float>(j,i) = ( 1.0 / pow(kernel_size,2)) ;
       }


   Mat filtered_image = convolve(input_image, kernel);  

   return filtered_image;
}

Mat morphological_filter(const Mat& input_image, int kernel_size, bool max, bool ignore_center_pixel=false )
{
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

  Mat filtered_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);

  int size = kernel_size*kernel_size;
  
  float neighborhood[size];

  int k;		

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
	   k = 0;	

	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{
		    int row = j+m-offset;
	  	    int col = i+n-offset;
		
		    if(ignore_center_pixel)
		    {		    
			    if(j!=row && i!=col)
			    {
				neighborhood[k] = padded_image.at<float>(row,col) ;
			        k++;
			    }
		     }
		    else
		     {
			neighborhood[k] = padded_image.at<float>(row,col) ;
			k++;

		     }

		}

	   if(ignore_center_pixel)
		merge_sort(neighborhood,0,size-2);
	   else
		merge_sort(neighborhood,0,size-1);
		

	   if(max)
	   {
		if(ignore_center_pixel)
			filtered_image.at<float>(j,i)  = neighborhood[k-2];
		else
			filtered_image.at<float>(j,i)  = neighborhood[k-1];
	   }	
	   else
		filtered_image.at<float>(j,i)  = neighborhood[0]; 

       }

  Mat depadded_image = image_depadding(filtered_image, offset);
  
  return depadded_image;

}


Mat median_filter(const Mat& input_image, int kernel_size)
{
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

  Mat filtered_image = Mat(padded_image.rows, padded_image.cols, CV_32F, 0.0);

  int size = kernel_size*kernel_size;
  float neighborhood[size];

  int k;

  int mid ;

   if(kernel_size % 2 != 0)
   { 
	mid = (size+1)/2-1;
   }
   else
   {
	mid = (size)/2-1;
   } 		

  float value = 0.0;
  for(int j = offset; j < padded_image.rows - offset ; j++)
       for(int i = offset; i < padded_image.cols - offset; i++)
       {
	   k = 0;	

	   for(int m = 0; m < kernel_size ; m++)
		for(int n = 0; n < kernel_size; n++)
		{
		
		    neighborhood[k] = padded_image.at<float>(j+m-offset,i+n-offset) ;
		    k++;
		}

	    merge_sort(neighborhood,0,size-1);

            filtered_image.at<float>(j,i)  = (float)neighborhood[mid]; 
       }

  Mat depadded_image = image_depadding(filtered_image, offset);
  
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

Mat image_padding_for_dft(const Mat& input_image)
{
   Mat padded_image ;
   int rows = getOptimalDFTSize(input_image.rows);
   int cols = getOptimalDFTSize(input_image.cols);
   copyMakeBorder(input_image, padded_image, 0, rows - input_image.rows, 0, cols - input_image.cols, BORDER_CONSTANT, Scalar::all(0));
   return padded_image;

}

Mat add_noise(const Mat& input_image, float std_dev)
{
	Mat noisy_image(input_image.rows,input_image.cols,CV_8U);
	Mat noise(input_image.rows,input_image.cols,CV_32F);
        randn(noise,Scalar::all(0),Scalar::all(std_dev));
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


Mat wiener_filter(const Mat& noisy_image, const Mat& signal_spectrum, Mat kernel, int kernel_size, float threshold, float std_dev)
{
   Mat noise(noisy_image.rows,noisy_image.cols,CV_8U);
   randn(noise,Scalar::all(0),Scalar::all(std_dev));
  
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
	    if( kernel_spectrum.at<float>(j,i) > threshold)
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


Mat inverse_filter(const Mat& noisy_image, Mat kernel, int kernel_size, float std_dev, float threshold, bool pseudo_inverse)
{
   Mat noise(noisy_image.rows,noisy_image.cols,CV_8U);
   randn(noise,Scalar::all(0),Scalar::all(std_dev));
  
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
		    if( kernel_spectrum.at<float>(j,i) > threshold)
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

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

//int main(int argc, char** argv )
//{
//    Mat image1, grayscale_image1, eroded_image1, dilated_image1,image2, grayscale_image2, eroded_image2, dilated_image2;

//    image1 = imread( "../images/letter_a.jpg", 1 );
//    image2 = imread( "../images/fingerprint.png", 1 );

//    if ( !image1.data || !image2.data )
//    {
//        printf("No image data \n");
//        return -1;
//    }

//    cvtColor( image1, grayscale_image1, CV_BGR2GRAY );
//    eroded_image1 = morphological_filter(grayscale_image1,3,false);
//    dilated_image1 = morphological_filter(grayscale_image1,3,true);

//    cvtColor( image2, grayscale_image2, CV_BGR2GRAY );
//    eroded_image2 = morphological_filter(grayscale_image2,3,false);
//    dilated_image2 = morphological_filter(grayscale_image2,3,true);

//    namedWindow("Original Image 1", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/grayscale_image1.jpg", grayscale_image1 );
//    imshow("Original Image 1", grayscale_image1);

//    namedWindow("Eroded Image 1", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/eroded_image1.jpg", eroded_image1 );
//    imshow("Eroded Image 1", eroded_image1 );

//    namedWindow("Dilated Image 1", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/dilated_image1.jpg", dilated_image1 );
//    imshow("Dilated Image 1", dilated_image1);

//    namedWindow("Original Image 2", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/grayscale_image2.jpg", grayscale_image2 );
//    imshow("Original Image 2", grayscale_image2);

//    namedWindow("Eroded Image 2", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/eroded_image2.jpg", eroded_image2 );
//    imshow("Eroded Image 2", eroded_image2 );

//    namedWindow("Dilated Image 2", WINDOW_AUTOSIZE);
//    imwrite( "../results/morphological_filter/dilated_image2.jpg", dilated_image2 );
//    imshow("Dilated Image 2", dilated_image2 );

//    waitKey(0);

//    return 0;
//}

