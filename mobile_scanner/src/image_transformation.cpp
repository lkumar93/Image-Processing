//
// THIS FILE CONTAINS VARIOUS FUNCTIONS FOR TRANSFORMING IMAGES
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

#include "image_transformation.h"

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

void bilinearInterpolation(Mat image, int vacant_pixel_value)
{
	  if(image.type() == CV_8UC3)
	  {
		  Vec3b vacant_pixel;
		  vacant_pixel[0] = vacant_pixel_value;
		  vacant_pixel[1] = vacant_pixel_value;
		  vacant_pixel[2] = vacant_pixel_value;

		 // Bilinear Interpolation

// 		   for(int j = 1; j < image.rows ; j++)
//		   {
//			for(int i = 1; i < image.cols; i++)
//			{
//				if(image.at<Vec3b>(j, i) == vacant_pixel)
//				{
//					int count_r_y = i+1;
//					int count_r_x = j+1;
//					//get bottom right filled_neighbour
//					while( image.at<Vec3b>(count_r_x, count_r_y) == vacant_pixel && count_r_y< image.cols-1 && count_r_x< image.rows-1)
//					{
//						count_r_y++;
//						count_r_x++;
//					}
//					int count_l_y = i-1;
//					int count_l_x = j-1;
//					//get top left most filled_neighbour
//					while( image.at<Vec3b>(count_l_x, count_l_y) == vacant_pixel && count_l_y > 0 && count_l_x > 0 )
//					{
//						count_l_x--;
//						count_l_y--;
//					}

//					int count_b_y = i-1;
//					int count_b_x = j+1;
//					//get bottom left filled_neighbour
//					while( image.at<Vec3b>(count_b_x, count_b_y) == vacant_pixel && count_b_x < image.rows-1 && count_b_y > 0 )
//					{
//						count_b_x++;
//						count_b_y--;
//					}

//					int count_t_x = j-1;
//					int count_t_y = i+1;
//					//get top right most filled_neighbour
//					while( image.at<Vec3b>(count_t_x, count_t_y) == vacant_pixel && count_t_x > 0  && count_t_y< image.cols-1)
//					{
//						count_t_x--;
//						count_t_y++;
//					}

////					if(count_r >=image.cols)
////						count_r = image.cols-1;

////					if(count_l < 0)
////						count_l = 0;

////					if(count_b >=image.rows)
////						count_b = image.rows-1;

////					if(count_t < 0)
////						count_t = 0;

//			
//					float left_offset = i-count_l_y;
//					float right_offset = count_r_y-i;

//					float top_offset = j-count_t_y;
//					float bottom_offset = count_b_y-j;

//					float col_offset = left_offset+right_offset;
//					float row_offset = top_offset+bottom_offset;

//					left_offset = (1-left_offset/col_offset)/2.0;
//					right_offset = (1-right_offset/col_offset)/2.0;
//	
//					top_offset = (1-top_offset/row_offset)/2.0;
//					bottom_offset = (1-bottom_offset/row_offset)/2.0;

//					Vec3b top_pixel = image.at<Vec3b>(count_t_x, count_t_y);
//					Vec3b bottom_pixel = image.at<Vec3b>(count_b_x, count_b_y);

//					Vec3b right_pixel = image.at<Vec3b>(count_r_x, count_r_y);
//					Vec3b left_pixel = image.at<Vec3b>(count_l_x, count_l_y);

//					Vec3b interpolated_value;

//					if(left_pixel == vacant_pixel)
//						left_offset = 0;

//					if(right_pixel == vacant_pixel)
//						right_offset = 0;

//					if(top_pixel == vacant_pixel)
//						top_offset = 0;

//					if(bottom_pixel == vacant_pixel)
//						bottom_offset = 0;

//					col_offset = left_offset+right_offset;
//					row_offset = top_offset+bottom_offset;

//					left_offset = left_offset/(2*col_offset);
//					right_offset = right_offset/(2*col_offset);

//					top_offset = top_offset/(2*row_offset);
//					bottom_offset = bottom_offset/(2*row_offset);
//				
//	
//					interpolated_value[0] = left_offset*left_pixel[0] + right_offset*right_pixel[0] + top_offset*top_pixel[0]
//									+ bottom_offset*bottom_pixel[0];

//					interpolated_value[1] = left_offset*left_pixel[1] + right_offset*right_pixel[1] + top_offset*top_pixel[1]
//								+ bottom_offset*bottom_pixel[1];

//					interpolated_value[2] = left_offset*left_pixel[2] + right_offset*right_pixel[2] + top_offset*top_pixel[2]
//								+ bottom_offset*bottom_pixel[2];

//					image.at<Vec3b>(j, i) = interpolated_value;

//					if(image.at<Vec3b>(j,i) == vacant_pixel)
//					{
//						std::cout<<"still vacant ="<<interpolated_value<<endl;
//					}	

//				}
//			}
//		   }

		   for(int j = 0; j < image.rows ; j++)
		   {
			for(int i = 0; i < image.cols; i++)
			{
				if(image.at<Vec3b>(j, i) == vacant_pixel)
				{
					int count_r = i;
					//get right most filled_neighbour
					while( image.at<Vec3b>(j, count_r) == vacant_pixel && count_r< image.cols)
					{
						count_r++;
					}
					int count_l = i;
					//get left most filled_neighbour
					while( image.at<Vec3b>(j, count_l) == vacant_pixel && count_l >= 0 )
					{
						count_l--;
					}

					int count_b = j;
					//get bottom most filled_neighbour
					while( image.at<Vec3b>(count_b, i) == vacant_pixel && count_b < image.rows )
					{
						count_b++;
					}

					int count_t = j;
					//get top most filled_neighbour
					while( image.at<Vec3b>(count_t, i) == vacant_pixel && count_t >= 0 )
					{
						count_t--;
					}

					if(count_r >=image.cols)
						count_r = image.cols-1;

					if(count_l < 0)
						count_l = 0;

					if(count_b >=image.rows)
						count_b = image.rows-1;

					if(count_t < 0)
						count_t = 0;

			
					float left_offset = i-count_l;
					float right_offset = count_r-i;

					float top_offset = j-count_t;
					float bottom_offset = count_b-j;

					float col_offset = left_offset+right_offset;
					float row_offset = top_offset+bottom_offset;

					if(col_offset > 0)
					{
						left_offset = (1-left_offset/col_offset)/2.0;
						right_offset = (1-right_offset/col_offset)/2.0;
					}
	
					if(row_offset >0)
					{
						top_offset = (1-top_offset/row_offset)/2.0;
						bottom_offset = (1-bottom_offset/row_offset)/2.0;
					}

					Vec3b top_pixel = image.at<Vec3b>(count_t, i);
					Vec3b bottom_pixel = image.at<Vec3b>(count_b, i);

					Vec3b right_pixel = image.at<Vec3b>(j, count_r);
					Vec3b left_pixel = image.at<Vec3b>(j, count_l);

					Vec3b interpolated_value;

					if(left_pixel == vacant_pixel)
						left_offset = 0;

					if(right_pixel == vacant_pixel)
						right_offset = 0;

					if(top_pixel == vacant_pixel)
						top_offset = 0;

					if(bottom_pixel == vacant_pixel)
						bottom_offset = 0;

					col_offset = left_offset+right_offset;
					row_offset = top_offset+bottom_offset;

					if(col_offset > 0)
					{
						left_offset = left_offset/(2*col_offset);
						right_offset = right_offset/(2*col_offset);
					}


					if(row_offset >0)
					{
						top_offset = top_offset/(2*row_offset);
						bottom_offset = bottom_offset/(2*row_offset);
					}

				
					float total = left_offset + right_offset + top_offset +bottom_offset;

					if(total > 0)
					{
						left_offset/=total;
						right_offset/=total;
						top_offset/=total;
						bottom_offset/=total;	
					}			
	
					interpolated_value[0] = left_offset*left_pixel[0] + right_offset*right_pixel[0] + top_offset*top_pixel[0]
									+ bottom_offset*bottom_pixel[0];

					interpolated_value[1] = left_offset*left_pixel[1] + right_offset*right_pixel[1] + top_offset*top_pixel[1]
								+ bottom_offset*bottom_pixel[1];

					interpolated_value[2] = left_offset*left_pixel[2] + right_offset*right_pixel[2] + top_offset*top_pixel[2]
								+ bottom_offset*bottom_pixel[2];

					image.at<Vec3b>(j, i) = interpolated_value;

				}
			}
		   }
	}
	else if(image.type() == CV_8UC1)
	{

		 // Bilinear Interpolation
		   for(int j = 0; j < image.rows ; j++)
		   {
			for(int i = 0; i < image.cols; i++)
			{
				if(image.at<uchar>(j, i) == vacant_pixel_value)
				{
					int count_r = i;
					//get right most filled_neighbour
					while( image.at<uchar>(j, count_r) == vacant_pixel_value && count_r< image.cols)
					{
						count_r++;
					}
					int count_l = i;
					//get left most filled_neighbour
					while( image.at<uchar>(j, count_l) == vacant_pixel_value && count_l >= 0 )
					{
						count_l--;
					}

					int count_b = j;
					//get bottom most filled_neighbour
					while( image.at<uchar>(count_b, i) == vacant_pixel_value && count_b < image.rows )
					{
						count_b++;
					}

					int count_t = j;
					//get top most filled_neighbour
					while( image.at<uchar>(count_t, i) == vacant_pixel_value && count_t >= 0 )
					{
						count_t--;
					}

					if(count_r >=image.cols)
						count_r = image.cols-1;

					if(count_l < 0)
						count_l = 0;

					if(count_b >=image.rows)
						count_b = image.rows-1;

					if(count_t < 0)
						count_t = 0;

			
					float left_offset = i-count_l;
					float right_offset = count_r-i;

					float top_offset = j-count_t;
					float bottom_offset = count_b-j;

					float col_offset = left_offset+right_offset;
					float row_offset = top_offset+bottom_offset;

					if(col_offset > 0)
					{
						left_offset = (1-left_offset/col_offset)/2.0;
						right_offset = (1-right_offset/col_offset)/2.0;
					}
	
					if(row_offset >0)
					{
						top_offset = (1-top_offset/row_offset)/2.0;
						bottom_offset = (1-bottom_offset/row_offset)/2.0;
					}

					int top_pixel = image.at<uchar>(count_t, i);
					int bottom_pixel = image.at<uchar>(count_b, i);

					int right_pixel = image.at<uchar>(j, count_r);
					int left_pixel = image.at<uchar>(j, count_l);

					int interpolated_value;

					if(left_pixel == vacant_pixel_value)
						left_offset = 0;

					if(right_pixel == vacant_pixel_value)
						right_offset = 0;

					if(top_pixel == vacant_pixel_value)
						top_offset = 0;

					if(bottom_pixel == vacant_pixel_value)
						bottom_offset = 0;

					if(col_offset > 0)
					{
						left_offset = left_offset/(2*col_offset);
						right_offset = right_offset/(2*col_offset);
					}


					if(row_offset >0)
					{
						top_offset = top_offset/(2*row_offset);
						bottom_offset = bottom_offset/(2*row_offset);
					}

				
					float total = left_offset + right_offset + top_offset +bottom_offset;

					if(total > 0)
					{
						left_offset/=total;
						right_offset/=total;
						top_offset/=total;
						bottom_offset/=total;	
					}			
	
	
					interpolated_value = left_offset*left_pixel + right_offset*right_pixel + top_offset*top_pixel
									+ bottom_offset*bottom_pixel;

					image.at<uchar>(j, i) = interpolated_value;	
				}
			}
		   }
	}
}

Mat perspective_transform(Point2f input_rectangle[], Point2f output_rectangle[])
{
	Mat A = Mat::ones( 3, 3, CV_32F );
	Mat B = Mat::ones( 3, 1, CV_32F );

	A.at<float>(0,0) = input_rectangle[0].x;
	A.at<float>(1,0) = input_rectangle[0].y;

	A.at<float>(0,1) = input_rectangle[1].x;
	A.at<float>(1,1) = input_rectangle[1].y;

	A.at<float>(0,2) = input_rectangle[2].x;
	A.at<float>(1,2) = input_rectangle[2].y;

	B.at<float>(0,0) = input_rectangle[3].x;
	B.at<float>(1,0) = input_rectangle[3].y;

	Mat C = Mat::ones( 3, 1, CV_32F );
	C = A.inv()*B;

	Mat M = Mat::ones( 3, 3, CV_32F );

	M.at<float>(0,0) = C.at<float>(0,0)*input_rectangle[0].x;
	M.at<float>(1,0) =  C.at<float>(0,0)*input_rectangle[0].y;
	M.at<float>(2,0) =  C.at<float>(0,0);

	M.at<float>(0,1) =  C.at<float>(1,0)* input_rectangle[1].x;
	M.at<float>(1,1) =  C.at<float>(1,0)*input_rectangle[1].y;
	M.at<float>(2,1) =  C.at<float>(1,0);

	M.at<float>(0,2) =  C.at<float>(2,0)*input_rectangle[2].x;
	M.at<float>(1,2) =  C.at<float>(2,0)*input_rectangle[2].y;
	M.at<float>(2,2) =  C.at<float>(2,0);

	Mat D = Mat::ones( 3, 3, CV_32F );
	Mat E = Mat::ones( 3, 1, CV_32F );

	D.at<float>(0,0) = output_rectangle[0].x;
	D.at<float>(1,0) = output_rectangle[0].y;

	D.at<float>(0,1) = output_rectangle[1].x;
	D.at<float>(1,1) = output_rectangle[1].y;

	D.at<float>(0,2) = output_rectangle[2].x;
	D.at<float>(1,2) = output_rectangle[2].y;

	E.at<float>(0,0) = output_rectangle[3].x;
	E.at<float>(1,0) = output_rectangle[3].y;

	Mat F = Mat::ones( 3, 1, CV_32F );
	F = D.inv()*E;

	Mat N = Mat::ones( 3, 3	, CV_32F );

	N.at<float>(0,0) = F.at<float>(0,0)*output_rectangle[0].x;
	N.at<float>(1,0) = F.at<float>(0,0)*output_rectangle[0].y;
	N.at<float>(2,0) = F.at<float>(0,0);

	N.at<float>(0,1) = F.at<float>(1,0)*output_rectangle[1].x;
	N.at<float>(1,1) = F.at<float>(1,0)*output_rectangle[1].y;
	N.at<float>(2,1) = F.at<float>(1,0);

	N.at<float>(0,2) = F.at<float>(2,0)*output_rectangle[2].x;
	N.at<float>(1,2) = F.at<float>(2,0)*output_rectangle[2].y;
	N.at<float>(2,2) = F.at<float>(2,0);


	Mat perspective_transform= Mat::ones( 3, 3, CV_32F );
	perspective_transform = N*M.inv();

	return perspective_transform;
}

Mat resize_image(const Mat& input_image, int width, int height)
{
	Mat resized_image ;

	float scale_x = (float)height/(float)input_image.rows;
	float scale_y = (float)width/(float)input_image.cols;

	if(input_image.type() == CV_8UC3)
	{
		resized_image = Mat::ones(height,width,CV_8UC3)*0;

		for( int j = 0; j <  input_image.rows ; j++ )
	   	    for( int i = 0; i <  input_image.cols; i++ )
		    {
			int x = cvFloor(scale_x*j);
			int y = cvFloor(scale_y*i);

			if(x >= 0 && x < height && y >=0 && y< width)
				resized_image.at<Vec3b>(x,y) = input_image.at<Vec3b>(j,i);

		    }
	}
	else if(input_image.type() == CV_8UC1)
	{
		resized_image = Mat::ones(height,width,CV_8UC1)*0;

		for( int j = 0; j <  input_image.rows ; j++ )
	   	    for( int i = 0; i <  input_image.cols; i++ )
		    {
			int x = cvFloor(scale_x*j);
			int y = cvFloor(scale_y*i);

			if(x >= 0 && x < height && y >=0 && y< width)
				resized_image.at<uchar>(x,y) = input_image.at<uchar>(j,i);

		    }
	}

	else
	{
		cout<<"Resize operation not available for this image type"<<endl;
		return resized_image;
	}

	bilinearInterpolation(resized_image,0);

	return resized_image;

}

void warp_image(const Mat& input_image, Mat& output_image, const Mat& perspective_transform)
{

	   Mat given_point = Mat::ones(3,1,CV_32F); 
	   Mat transformed_point = Mat::ones(3,1,CV_32F);

	  if(input_image.type() == CV_8UC3)
	  {
		   for( int j = 0; j <  input_image.rows ; j++ )
	   	      for( int i = 0; i <  input_image.cols; i++ )
			{
				given_point.at<float>(0,0) = i;
				given_point.at<float>(1,0) = j;

				transformed_point = perspective_transform*given_point;

				int x = cvFloor(transformed_point.at<float>(1,0)/transformed_point.at<float>(2,0));
				int y = cvFloor(transformed_point.at<float>(0,0)/transformed_point.at<float>(2,0));

				if(x >= 0 && x < output_image.rows && y >=0 && y< output_image.cols)
					output_image.at<Vec3b>(x,y) = input_image.at<Vec3b>(j,i);

			}

	  }
	  else if(input_image.type() == CV_8UC1)
	  {
		   for( int j = 0; j <  input_image.rows ; j++ )
	   	      for( int i = 0; i <  input_image.cols; i++ )
			{
				given_point.at<float>(0,0) = i;
				given_point.at<float>(1,0) = j;

				transformed_point = perspective_transform*given_point;

				int x = cvFloor(transformed_point.at<float>(1,0)/transformed_point.at<float>(2,0));
				int y = cvFloor(transformed_point.at<float>(0,0)/transformed_point.at<float>(2,0));

				if(x >= 0 && x < output_image.rows && y >=0 && y< output_image.cols)
					output_image.at<uchar>(x,y) = input_image.at<uchar>(j,i);

			}
	   }

}

