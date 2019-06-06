#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <opencv2/highgui/highgui.hpp>
#include <labeling_geon.h>
#include <math.h>

#include <iostream>  
#include <string>
#include <sstream>
#include <opencv2/core/mat.hpp>  
#include <opencv2/imgcodecs.hpp>   
  
using namespace cv;  
using namespace std;  

CLabeling label;
static const std::string OPENCV_WINDOW = "IMAGE WINDOW";
void transmission_data(std_msgs::String);
cv::Mat img_proc(cv::Mat);
cv::Mat output_img;


CLabeling::CLabeling(void)
{
};

CLabeling::~CLabeling(void)
{
};



class ImageConverter
{
  ros::NodeHandle nh_;
  //ServiceClient sc;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/hsv_color_filter/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
	output_img=cv::Mat::zeros(240,320,CV_8UC3);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
 
 
 void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
	try
    {
     	 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   	 }
   	 catch (cv_bridge::Exception& e)
   	 {
     	 ROS_ERROR("cv_bridge exception: %s", e.what());
     	 return;
   	 }
	cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(320,240),0,0,CV_INTER_NN);
	output_img=cv_ptr->image;
	cv_ptr->image=img_proc(cv_ptr->image);
	cv::waitKey(3);
    
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(150, 150), 100, CV_RGB(255,0,0));
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
	
	
  cv::Mat img_proc(cv::Mat src_mat)
	{
		cv::Mat dst_mat(cv::Size(src_mat.size()), src_mat.type());


		if(label.CLabeling::set_Label(src_mat, 1000) )
		{
			if(label.set_MaxLabel())
			{
				label.draw_MaxLabel(output_img,cv::Scalar(255,255,0),1);
				ROS_INFO_STREAM(label.get_MaxLabel_FirstPt().x);	
			}
			else
				ROS_INFO_STREAM("Finding the labeling fail!" );	
		}
		else
			ROS_INFO_STREAM("Detection fail");

	  	DrawLabelingImage(output_img);
	  
		cv::imshow(OPENCV_WINDOW, output_img);
		//transmission_data(command);

		return dst_mat;	
	}
	
	void DrawLabelingImage(Mat image)
{
	Mat img_gray, img_color, img_binary;
	
	cvtColor(image, img_gray, COLOR_BGR2GRAY);	// Convert the image to Gray
	threshold(img_gray, img_binary, 127, 255, THRESH_BINARY);
	cvtColor(img_gray, img_color, COLOR_GRAY2BGR);


	Mat img_labels, stats, centroids;
	int numOfLables = connectedComponentsWithStats(img_binary, img_labels,
		stats, centroids, 8, CV_32S);


	//라벨링된 이미지중 특정 라벨을 컬러로 표현해주기  
	for (int y = 0; y<img_labels.rows; ++y) {

		int *label = img_labels.ptr<int>(y);
		Vec3b* pixel = img_color.ptr<Vec3b>(y);


		for (int x = 0; x < img_labels.cols; ++x) {


			if (label[x] == 3) {
				pixel[x][2] = 0;
				pixel[x][1] = 255;
				pixel[x][0] = 0;
			}
		}
	}

	int max_idx, max_size = 0;
	//라벨링 된 이미지에 각각 직사각형으로 둘러싸기  
	for (int j = 1; j < numOfLables; j++) 
	{
		int area = stats.at<int>(j, CC_STAT_AREA);
		int left = stats.at<int>(j, CC_STAT_LEFT);
		int top = stats.at<int>(j, CC_STAT_TOP);
		int width = stats.at<int>(j, CC_STAT_WIDTH);
		int height = stats.at<int>(j, CC_STAT_HEIGHT);

		int x = centroids.at<double>(j, 0); //중심좌표
		int y = centroids.at<double>(j, 1);

		// Find the maximum rect
		if(width*height >= max_size)
		{
			max_idx = j;
			max_size = width*height;
		}
		/*
		circle(img_color, Point(x, y), 5, Scalar(255, 0, 0), 1);

		rectangle(img_color, Point(left, top), Point(left + width, top + height),
			Scalar(0, 0, 255), 1);

		 //convert integer to string
		  std::ostringstream os;
		  os << j;
		  std::cout << os.str() << std::endl;

		putText(img_color, os.str(), Point(left + 20, top + 20), FONT_HERSHEY_SIMPLEX,	1, Scalar(255, 0, 0), 2);
		*/
	}
	
	
	int area = stats.at<int>(max_idx, CC_STAT_AREA);
	int left = stats.at<int>(max_idx, CC_STAT_LEFT);
	int top = stats.at<int>(max_idx, CC_STAT_TOP);
	int width = stats.at<int>(max_idx, CC_STAT_WIDTH);
	int height = stats.at<int>(max_idx, CC_STAT_HEIGHT);

	int x = centroids.at<double>(max_idx, 0); //중심좌표
	int y = centroids.at<double>(max_idx, 1);
	circle(img_color, Point(x, y), 5, Scalar(255, 0, 0), 1);
	rectangle(img_color, Point(left, top), Point(left + width, top + height),
			Scalar(0, 0, 255), 1);

	//convert integer to string
	std::ostringstream os;
	os << max_idx;
	std::cout << os.str() << std::endl;
	putText(img_color, os.str(), Point(left + 20, top + 20), FONT_HERSHEY_SIMPLEX,	1, Scalar(255, 0, 0), 2);
	
	
	namedWindow("Labeling Image", WINDOW_AUTOSIZE);				// Create a window for display
	imshow("Labeling Image", img_color);						// Show our image inside it
}
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


