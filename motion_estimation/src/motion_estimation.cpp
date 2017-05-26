//
// THIS FILE CONTAINS THE IMPLEMENTATION OF BLOCK MATCHING BASED MOTION
// ESTIMATION TECHNIQUE

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

#include "motion_estimation.h"
#include "image_transformation.h"

///////////////////////////////////////////
//
//	MEMBER FUNCTIONS
//
///////////////////////////////////////////

MotionField::MotionField(int num_rows, int num_cols, Mat previous_frame)
{
	rows = num_rows;
	cols = num_cols;
	height = num_rows;
	width = num_cols;
	motion_field.resize(rows*cols);
	motion_field_image = previous_frame.clone();
}

void MotionField::set(int row, int col, const MotionVector& motion_vector)
{

	if(row >= rows || row < 0)
	{
		cout<<"Row index out of bounds of the height of motion field"<<endl;
		return ;
	}

	if(col >= cols || col < 0 )
	{
		cout<<"Col index out of bounds of the width of motion field"<<endl;
		return;
	}

	int index = row*cols + col;
	motion_field[index] = motion_vector;
}

MotionVector& MotionField::get(int row, int col)
{
	if(row >= rows || row < 0)
	{
		cout<<"Row index out of bounds of the height of motion field"<<endl;
		row = rows-1;
	}

	if(col >= cols || col < 0 )
	{
		cout<<"Col index out of bounds of the width of motion field"<<endl;
		col = cols-1;
	}
	return motion_field[row*cols+col];
}

void MotionField::print(int resolution)
{
	cout << "Printing Motion Field"<<endl;
	for(int j = 0; j < rows ; j+=resolution)
	{
		for(int i = 0; i < cols; i+=resolution)
		{
		   MotionVector motion_vector = get(j,i);
		   float dx = motion_vector.vel_x;
		   float dy = motion_vector.vel_y;

		   cout<<"("<<dx<<","<<dy<<") ";
		}
	cout<<endl;
	}
}

void MotionField::quiver(const Mat& image, int resolution)
{

   float max_length = -1000;

   for(int j = 0; j < rows ; j+=resolution)
	for(int i = 0; i < cols; i+=resolution)
	{
	   MotionVector motion_vector = get(j,i);
	   float dx = motion_vector.vel_x;
	   float dy = motion_vector.vel_y;

	   float length = sqrt(pow(dx,2)+pow(dy,2));
	
	   if(length > max_length)
		max_length = length;

	}


   for(int j = 0; j < rows ; j+=resolution)
	for(int i = 0; i < cols; i+=resolution)
	{
	   MotionVector motion_vector = get(j,i);
	   float dx = motion_vector.vel_x;
	   float dy = motion_vector.vel_y;

	   cv::Point p1(i,j);
	
	   float length = sqrt(pow(dx,2)+pow(dy,2));

	   if(length > 0)
	   {
		float arrow_size = 5.0*length/max_length;
		
		cv::Point p2((int)(p1.x+dx),(int)(p1.y+dy));

		arrowedLine(image,p1,p2,CV_RGB(0,255,0),1,CV_AA,0,0.2);

	   }

	   else
	   {
		circle(  image, Point( i, j ), 0.2,  Scalar(0,255,0), 0.1, 8, 0 );
	   }

	}

}

std::vector<MotionVector> MotionField::getMotionField()
{
	return motion_field;
}

Mat MotionField::getImage(int resolution)
{
	quiver(motion_field_image,resolution);
	return motion_field_image;

}

///////////////////////////////////////////
//
//	FUNCTIONS
//
///////////////////////////////////////////

MotionField estimateMotionByBlockMatching(const Mat& previousFrame, const Mat& currentFrame,int matching_criteria, int block_size, int search_window_size)
{

	Mat previous_frame_resized, current_frame_resized;

	float width_offset = (float)currentFrame.cols/(float)block_size;
	int new_width = currentFrame.cols + (cvRound(width_offset) - width_offset)*block_size;

	float height_offset = (float)currentFrame.rows/(float)block_size;
	int new_height = currentFrame.rows + (cvRound(height_offset) - height_offset)*block_size;

	if(new_width == currentFrame.cols && new_height == currentFrame.rows )
	{
		previous_frame_resized = previousFrame.clone();
		current_frame_resized = currentFrame.clone();
	}

	else
	{
		resize(previousFrame, previous_frame_resized, Size(new_height,new_width));
		resize(currentFrame, current_frame_resized, Size(new_height,new_width));
	}

	Mat previous_grayscale_image = Mat::zeros( new_height, new_width, CV_8UC1 );
	Mat current_grayscale_image = Mat::zeros( new_height, new_width, CV_8UC1 );


	cvtColor( previous_frame_resized , previous_grayscale_image, CV_BGR2GRAY );
	cvtColor( current_frame_resized , current_grayscale_image, CV_BGR2GRAY );

	MotionField motion_field(new_height, new_width, previous_frame_resized);

	if(search_window_size <= block_size || search_window_size%block_size != 0)
	{
		cout<<"Search window size must be greater than the block size and must be a multiple of block size  ";
		return motion_field;
	}


	int number_of_blocks_cols = new_width/block_size;

	int number_of_blocks_rows = new_height/block_size;

	cv::Mat currentFrameBlocks[number_of_blocks_rows][number_of_blocks_cols];

 	//Parse through each block in current image
	for(int j = 0; j < number_of_blocks_rows ; j++)
	{
		for(int i = 0; i < number_of_blocks_cols ; i++)
		{

			currentFrameBlocks[j][i] = Mat::zeros( block_size, block_size, CV_8UC1 );

			int start_row_index = j*block_size;
			int start_col_index = i*block_size;	

			int stop_row_index = start_row_index +block_size;
			int stop_col_index = start_col_index +block_size;

			if(start_row_index < 0)
				start_row_index = 0;

			if(start_col_index < 0)
				start_col_index = 0;

			if(stop_row_index > new_height)
				stop_row_index = new_height;

			if(stop_col_index > new_width)
				stop_col_index = new_width;
			

			int row = 0;
			int col = 0;

			//Filling up each block with the values in current frame
			for(int m=start_row_index; m < stop_row_index; m++)
			{
				col = 0;
				for(int n=start_col_index ; n < stop_col_index; n++)
				{
					currentFrameBlocks[j][i].at<uchar>(row,col) = current_grayscale_image.at<uchar>(m,n);
					col++;
				}
				row++;
			}

			int center_x = (start_row_index+stop_row_index)/2;
			int center_y = (start_col_index+stop_col_index)/2;

			int start_row_search_index = center_x - search_window_size/2 ;
			int start_col_search_index = center_y - search_window_size/2 ;	

			int stop_row_search_index = center_x + search_window_size/2;
			int stop_col_search_index = center_y + search_window_size/2;

			if(start_row_search_index < 0)
				start_row_search_index = 0;

			if(start_col_search_index < 0)
				start_col_search_index = 0;

			if(stop_row_search_index > new_height)
				stop_row_search_index = new_height;

			if(stop_col_search_index > new_width)
				stop_col_search_index = new_width;

			float lowest_error = 10000;

			MotionVector best_motion_vector;

			//Parse through the search window in previous frame and find the best motion vector
			for(int m=start_row_search_index; m < stop_row_search_index; m++)
			{
				for(int n=start_col_search_index ; n < stop_col_search_index; n++)
				{

					float error = 0.0;

					//Compute either Sum of Squared Differences or Sum of Absolute Differences
					for(int q=0; q < block_size; q++)
					{
						for(int r=0 ; r < block_size; r++)
						{
		
							int current_frame_pixel = currentFrameBlocks[j][i].at<uchar>(q,r);

							int row_index = (m+q);
							int col_index = (r+n);

							int previous_frame_pixel = previous_grayscale_image.at<uchar>(m+q,r+n);

							if(matching_criteria == SAD)
								error += abs(current_frame_pixel - previous_frame_pixel);

							else
								error += sqrt(pow(current_frame_pixel-previous_frame_pixel,2));
							
						}
					}

					int center_x_search_window = m+block_size/2;
					int center_y_search_window = n+block_size/2;

					if(error<lowest_error)
					{
						lowest_error = error;
						best_motion_vector.vel_y = center_x - center_x_search_window; 
						best_motion_vector.vel_x = center_y - center_y_search_window;
					}				
				}
			}

			//Fill the values in motion field
			for(int m=start_row_index; m < stop_row_index; m++)
			{
				for(int n=start_col_index ; n < stop_col_index; n++)
				{
					motion_field.set(m,n,best_motion_vector);	

				}
			}							
		}
	}

	return motion_field;
}

float mean_squared_error(const Mat& image_1, const Mat& image_2)
{

   int m1 = image_1.rows;
   int n1 = image_1.cols;

   int m2 = image_2.rows;
   int n2 = image_2.cols;

   if(m1 != m2 || n1 != n2)
	resize(image_2,image_2,Size(n1,m1));

   float mse = 0.0;

    for(int j = 0; j < m1 ; j++)
        for(int i = 0; i < n1; i++)
        {
            Vec3b rgb_value_1 = image_1.at<Vec3b>(j, i);
	    Vec3b rgb_value_2 = image_2.at<Vec3b>(j, i);

	    mse += abs(rgb_value_1[0]-rgb_value_2[0])^2 + abs(rgb_value_1[1]-rgb_value_2[1])^2 + abs(rgb_value_1[2]-rgb_value_2[2])^2 ;
        }

    return mse/(m1*n1*3.0);
}

Mat compensateMotion(Mat previous_frame, MotionField motion_field)
{
	Mat previous_frame_resized;
	resize(previous_frame, previous_frame_resized, Size(motion_field.width,motion_field.height));

	Mat motion_compensated_image =  Mat(motion_field.height, motion_field.width, CV_8UC3, Scalar(0,0,0));

	   for(int j = 0; j < motion_field.height ; j++)
	   {
		for(int i = 0; i < motion_field.width; i++)
		{
			MotionVector motion_vector = motion_field.get(j,i);	
			int predicted_row = j+motion_vector.vel_y;
			int predicted_col = i+motion_vector.vel_x;

			if(predicted_row < motion_field.height && predicted_col<motion_field.width && predicted_row >= 0 && predicted_col >= 0)
				motion_compensated_image.at<Vec3b>(predicted_row, predicted_col) = previous_frame_resized.at<Vec3b>(j, i);
		}
	   }

 	bilinearInterpolation(motion_compensated_image,0);

	return motion_compensated_image;	
}

Mat compute_prediction_error(const Mat& predicted_image, const Mat& target_image)
{
	Mat target_image_resized,prediction_error_image,predicted_image_grayscale;
	resize(target_image, target_image_resized, predicted_image.size());
	prediction_error_image = Mat::zeros(predicted_image.rows,predicted_image.cols,CV_8UC3);

	for(int j = 0; j < predicted_image.rows ; j++)
	{
		for(int i = 0; i < predicted_image.cols; i++)
		{
			Vec3b target_image_pixel, predicted_image_pixel, prediction_error_pixel;
			target_image_pixel = target_image_resized.at<Vec3b>(j,i);
			predicted_image_pixel = predicted_image.at<Vec3b>(j,i);
			prediction_error_image.at<Vec3b>(j,i) = target_image_pixel - predicted_image_pixel;

		}
	}
	
	return prediction_error_image;
}
