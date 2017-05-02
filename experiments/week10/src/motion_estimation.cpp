#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

#define THRESHOLD 35000000000
#define PI 3.1416

#define SAD 0
#define SSD 1

using namespace cv;
using namespace std;

struct MotionVector
{
	float vel_x;
	float vel_y;

	MotionVector() :vel_x(0.0),vel_y(0.0){};
};


class MotionField
{
	std::vector<MotionVector> motion_field;
        cv::Mat motion_field_image;
	int matching_criteria;
	int rows;
	int cols;

  public:
	int height;
	int width;
	MotionField(int num_rows, int num_cols, const Mat& previous_frame)
	{
		rows = num_rows;
		cols = num_cols;
		height = num_rows;
		width = num_cols;
		motion_field.resize(rows*cols);
		motion_field_image = previous_frame.clone();
	}

	void set(int row, int col, const MotionVector& motion_vector)
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

	MotionVector& get(int row, int col)
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

	void print(int resolution=10)
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

	void quiver(const Mat& image, int resolution)
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

//			cv::line(image,p1,p2,CV_RGB(0,255,0),1,CV_AA);

//			float angle = atan2((float) p1.y - p2.y, (float) p1.x -p2.x);

//		        p1.x = (int) (p2.x + arrow_size * cos(angle + PI / 4));
//		        p1.y = (int) (p2.y + arrow_size * sin(angle + PI / 4));
//			cv::line(image,p1,p2,CV_RGB(0,255,0),1,CV_AA,0);

//		        p1.x = (int) (p2.x + arrow_size * cos(angle - PI / 4));
//		        p1.y = (int) (p2.y + arrow_size * sin(angle - PI / 4));
//			cv::line(image,p1,p2,CV_RGB(0,255,0),1,CV_AA,0);
		   }

		   else
		   {
			circle(  image, Point( i, j ), 0.2,  Scalar(0,255,0), 0.1, 8, 0 );
		   }

		}

	}

	std::vector<MotionVector> getMotionField()
	{
		return motion_field;
	}

	Mat getImage(int resolution=10)
	{
		quiver(motion_field_image,resolution);
		return motion_field_image;

	}

};

MotionField estimateMotionByBlockMatching(const Mat& previousFrame, const Mat& currentFrame,int matching_criteria = SAD, int block_size = 8, int search_window_size = 32)
{

	Mat previous_frame_resized, current_frame_resized;

	float width_offset = (float)currentFrame.cols/(float)block_size;
	int new_width = currentFrame.cols + (cvRound(width_offset) - width_offset)*block_size;

	float height_offset = (float)currentFrame.rows/(float)block_size;
	int new_height = currentFrame.rows + (cvRound(height_offset) - height_offset)*block_size;

	Mat previous_grayscale_image, current_grayscale_image;

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

//			cout<<"current_center x = "<<center_x<<" , y="<<center_y<<endl;

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
							int previous_frame_pixel = previous_grayscale_image.at<uchar>(m+q,r+n);

							if(matching_criteria == SAD)
								error += abs(current_frame_pixel - previous_frame_pixel);

							else
								error += sqrt(pow(current_frame_pixel-previous_frame_pixel,2));

							//cout<<"current index q="<<q<<", r= "<<r<<" , value="<<current_frame_pixel<<" previous index m+q="<<m+q<<" ,r+n="<<r+n<<" ,value ="<<previous_frame_pixel<<endl;
							
						}
					}

					int center_x_search_window = m+block_size/2;
					int center_y_search_window = n+block_size/2;

					//cout<<"search window x = "<<center_x_search_window<<" , y="<<center_y_search_window<<endl;
					//cout<<"block x = "<<center_x<<" , y="<<center_y<<" , ssd="<<ssd<<endl;


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

			//cout<<"best motion_vector ssd = "<<best_ssd<<" x ="<<best_motion_vector.vel_x<<" , y= "<<best_motion_vector.vel_y<<endl;

			//cout<<"block j="<<j<<" , i= "<<i<<endl;

			//return;

							
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

	Mat motion_compensated_image =  Mat(motion_field.height, motion_field.width, CV_8UC3, Scalar(0,0,0));//Mat::zeros( motion_field.height, motion_field.width, CV_8UC3 )*-1;

	   for(int j = 0; j < motion_field.height ; j++)
	   {
		for(int i = 0; i < motion_field.width; i++)
		{
			MotionVector motion_vector = motion_field.get(j,i);	
			int predicted_row = j+motion_vector.vel_y;
			int predicted_col = i+motion_vector.vel_x;

			//cout<<"row ="<<predicted_row<<" ,col ="<<predicted_col<<endl;
			if(predicted_row < motion_field.height && predicted_col<motion_field.width)
				motion_compensated_image.at<Vec3b>(predicted_row, predicted_col) = previous_frame_resized.at<Vec3b>(j, i);
		}
	   }

	  Vec3b unfilled_neighbour;
	  unfilled_neighbour[0] = 0;
	  unfilled_neighbour[1] = 0;
	  unfilled_neighbour[2] = 0;

	 // Bilinear Interpolation
	   for(int j = 0; j < motion_field.height ; j++)
	   {
		for(int i = 0; i < motion_field.width; i++)
		{
			if(motion_compensated_image.at<Vec3b>(j, i) == unfilled_neighbour)
			{
				int count_r = i;
				//get right most filled_neighbour
				while( motion_compensated_image.at<Vec3b>(j, count_r) == unfilled_neighbour && count_r<motion_field.width )
				{
					count_r++;
				}
				int count_l = i;
				//get left most filled_neighbour
				while( motion_compensated_image.at<Vec3b>(j, count_l) == unfilled_neighbour && count_l >= 0 )
				{
					count_l--;
				}

				int count_b = j;
				//get bottom most filled_neighbour
				while( motion_compensated_image.at<Vec3b>(count_b, i) == unfilled_neighbour && count_b < motion_field.height )
				{
					count_b++;
				}

				int count_t = j;
				//get top most filled_neighbour
				while( motion_compensated_image.at<Vec3b>(count_t, i) == unfilled_neighbour && count_t >= 0 )
				{
					count_t--;
				}

				if(count_r >=motion_field.width)
					count_r = motion_field.width;

				if(count_l < 0)
					count_l = 0;

				if(count_b >=motion_field.height)
					count_b = motion_field.height;

				if(count_t < 0)
					count_t = 0;

			
				float left_offset = i-count_l;
				float right_offset = count_r-i;

				float top_offset = j-count_t;
				float bottom_offset = count_b-j;

				float col_offset = left_offset+right_offset;
				float row_offset = top_offset+bottom_offset;

				left_offset = (1-left_offset/col_offset)/2.0;
				right_offset = (1-right_offset/col_offset)/2.0;
	
				top_offset = (1-top_offset/row_offset)/2.0;
				bottom_offset = (1-bottom_offset/row_offset)/2.0;

				Vec3b top_pixel = motion_compensated_image.at<Vec3b>(count_t, i);
				Vec3b bottom_pixel = motion_compensated_image.at<Vec3b>(count_b, i);

				Vec3b right_pixel = motion_compensated_image.at<Vec3b>(j, count_r);
				Vec3b left_pixel = motion_compensated_image.at<Vec3b>(j, count_l);

				Vec3b interpolated_value;

				if(left_pixel == unfilled_neighbour)
					left_offset = 0;

				if(right_pixel == unfilled_neighbour)
					right_offset = 0;

				if(top_pixel == unfilled_neighbour)
					top_offset = 0;

				if(bottom_pixel == unfilled_neighbour)
					bottom_offset = 0;

				col_offset = left_offset+right_offset;
				row_offset = top_offset+bottom_offset;

				left_offset = left_offset/(2*col_offset);
				right_offset = right_offset/(2*col_offset);

				top_offset = top_offset/(2*row_offset);
				bottom_offset = bottom_offset/(2*row_offset);
				
	
				interpolated_value[0] = left_offset*left_pixel[0] + right_offset*right_pixel[0] + top_offset*top_pixel[0]
							+ bottom_offset*bottom_pixel[0];

				interpolated_value[1] = left_offset*left_pixel[1] + right_offset*right_pixel[1] + top_offset*top_pixel[1]
							+ bottom_offset*bottom_pixel[1];

				interpolated_value[2] = left_offset*left_pixel[2] + right_offset*right_pixel[2] + top_offset*top_pixel[2]
							+ bottom_offset*bottom_pixel[2];

				motion_compensated_image.at<Vec3b>(j, i) = interpolated_value;

				cout<<interpolated_value<<endl;
			}
			//motion_compensated_image.at<Vec3b>(j, i) = previous_frame_resized.at<Vec3b>(j, i);
		}
	   }

	return motion_compensated_image;	

}


int main(int argc, char** argv )
{
    Mat grayscale_image1, grayscale_image2;

    Mat image1 = imread( "../images/foreman/fm0001.tif", 1 );
    Mat image2 = imread( "../images/foreman/fm0002.tif", 1 );
    if ( !image1.data || !image2.data )
     {
        printf("No image data \n");
        return -1;
     }

   MotionField motion_field = estimateMotionByBlockMatching(image1,image2,SAD);

   Mat motion_estimated_image = motion_field.getImage();

   Mat motion_compensated_image = compensateMotion(image1,motion_field);

   float mse = mean_squared_error(motion_compensated_image, image2);

   cout<<"mse ="<<mse<<endl;

   //motion_field.print();

   namedWindow("Estimated Motion Field", WINDOW_AUTOSIZE);
   imshow("Estimated Motion Field", motion_estimated_image); 
   imwrite( "../results/motion_estimation/foreman/sad/estimated_motion_field.jpg", motion_estimated_image);

   namedWindow("Motion Compensated Image", WINDOW_AUTOSIZE);
   imshow("Motion Compensated Image", motion_compensated_image); 
   imwrite( "../results/motion_compensation/foreman/sad/motion_compensated_image.jpg", motion_compensated_image);

    waitKey(0);

    return 0;
}
