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
  image_transport::Subscriber bg_image_sub_;
  image_transport::Subscriber object_image_sub_;
  image_transport::Subscriber raw_image_sub_;
  image_transport::Publisher roi_image_pub_;
  image_transport::Publisher result_image_pub_;
  int max_x, max_y, max_w, max_h;
  int roi_x, roi_y, roi_w, roi_h;
  int getROI_flg;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    raw_image_sub_ = it_.subscribe("/camera1/color/image_raw", 1, &ImageConverter::rawImageCb, this);
    bg_image_sub_ = it_.subscribe("/hsv_color_filter/image", 1, &ImageConverter::bgImageCb, this);
    object_image_sub_ = it_.subscribe("/hsv_color_two_filter/image", 1, &ImageConverter::objImageCb, this);
    roi_image_pub_ = it_.advertise("/image_converter/output_video", 1);
    result_image_pub_ = it_.advertise("/image_converter/result", 1);

	output_img=cv::Mat::zeros(240,320,CV_8UC3);	
	max_x, max_y, max_w, max_h = 0, 0, 0, 0;
	roi_x, roi_y, roi_w, roi_h = 0, 0, 0, 0;
 	getROI_flg = false;
  }

  ~ImageConverter()
  {
    cv::destroyAllWindows();
  }
 
  void rawImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	 if(!getROI_flg)
	 	return ;
	  
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
	cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(320,240),0,0,CV_INTER_NN);	  
		 
	//Show ROI image abouy object(enclosure)
	if( roi_x + roi_w < cv_ptr->image.size().width && roi_y + roi_h < cv_ptr->image.size().height )
		cv_ptr->image = setROI(cv_ptr->image, roi_x, roi_y, roi_w, roi_h);
	else 
		ROS_INFO_STREAM( roi_x << ", " << roi_y << ", " << roi_w <<  ", " << roi_h);	
	
	cv::waitKey(3);  
	// Send the raw image for processing second HSV node.
    roi_image_pub_.publish(cv_ptr->toImageMsg());  
	getROI_flg = false;
  }
	
  void objImageCb(const sensor_msgs::ImageConstPtr& msg)
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
	  
	namedWindow("Object Labeling Image", WINDOW_AUTOSIZE);				// Create a window for display
	imshow("Object Labeling Image", getObjectImage(cv_ptr->image));			// Show our image inside it
	
	ROS_INFO_STREAM(roi_x << ", " << roi_y << ", " << roi_w <<  ", " << roi_h);
	  
	//Object center point
	ROS_INFO_STREAM( "Center_x: " << roi_x + max_x + max_w * 0.5 << ", Center_y "<< roi_y + max_y + max_h *0.5);
	
	cv::waitKey(3);  
    result_image_pub_.publish(cv_ptr->toImageMsg());  	  
  }
 
 void bgImageCb(const sensor_msgs::ImageConstPtr& msg)
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
	cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(320,240),0,0,CV_INTER_NN);
		 
	namedWindow("Conveyer Labeling Image", WINDOW_AUTOSIZE);				// Create a window for display
	imshow("Conveyer Labeling Image", img_main_proc(cv_ptr->image));						// Show our image inside it
	getROI_flg = true;
	 cv::waitKey(3);    
	//Draw circle
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(150, 150), 100, CV_RGB(255,0,0));
    // Output modified video stream
  }
	
	
  cv::Mat img_main_proc(cv::Mat src_mat)
	{
	  	cv::Mat out_img;
		/* Private library usage
		if(label.CLabeling::set_Label(src_mat, 1000) )
		{
			if(label.set_MaxLabel())
			{
				label.draw_MaxLabel(output_img,cvFIMA::Scalar(255,255,0),1);
				ROS_INFO_STREAM(label.get_MaxLabel_FirstPt().x);	
			}
			else
				ROS_INFO_STREAM("Finding the labeling fail!" );	
		}
		else
			ROS_INFO_STREAM("Detection fail");
		*/
	  
	  	src_mat = adjustFilter(src_mat);
	  
	  	//Labeling of maximum blob, 
	  	out_img = drawLabelingImage(src_mat);
	  
	  	return out_img;
	}
	
	cv::Mat adjustFilter(Mat src_img)
	{		
		  // 필터 효과를 더 두드러지게 5x5 구조 요소를 사용
		 cv::Mat element5(3, 3, CV_8U, cv::Scalar(1));
		 cv::Mat return_img;				

		 // 영상 닫힘과 영상 열림
		 cv::morphologyEx(src_img,return_img,cv::MORPH_CLOSE,element5);
		 cv::morphologyEx(return_img,return_img,cv::MORPH_OPEN,element5);
		 //cv::namedWindow("Closed Image");
		 //cv::imshow("Closed Image", return_img);
		
		return return_img;
	}
	
	cv::Mat setROI(Mat src_img, int X, int Y, int W, int H)
	{		
		// 관심영역 설정 (set ROI (X, Y, Width, Height)).
		Rect rect(X, Y, W, H);

		// 관심영역 자르기 (Crop ROI).
		Mat img_roi = src_img(rect);

		// show
		//imshow("image", img_roi);
		
		return img_roi;
	}
		
	
	cv::Mat getObjectImage(Mat image)
	{
		Mat img_gray, img_labeling, img_binary;

		cvtColor(image, img_gray, COLOR_BGR2GRAY);	// Convert the image to Gray
		threshold(img_gray, img_binary, 127, 255, THRESH_BINARY);
		cvtColor(img_gray, img_labeling, COLOR_GRAY2BGR);


		Mat img_labels, stats, centroids;
		int numOfLables = connectedComponentsWithStats(img_binary, img_labels,
			stats, centroids, 8, CV_32S);


		int max_idx, max_size = 0;
		//라벨링 된 이미지
		for (int j = 1; j < numOfLables; j++) 
		{
			int width = stats.at<int>(j, CC_STAT_WIDTH);
			int height = stats.at<int>(j, CC_STAT_HEIGHT);

			// Find the maximum rect
			if(width*height >= max_size)
			{
				max_idx = j;
				max_size = width*height;
			}
		}
		int area = stats.at<int>(max_idx, CC_STAT_AREA);
		int left = stats.at<int>(max_idx, CC_STAT_LEFT);
		int top = stats.at<int>(max_idx, CC_STAT_TOP);
		int width = stats.at<int>(max_idx, CC_STAT_WIDTH);
		int height = stats.at<int>(max_idx, CC_STAT_HEIGHT);

		int x = centroids.at<double>(max_idx, 0); //중심좌표
		int y = centroids.at<double>(max_idx, 1);
		
		// Draw circle on the center, rectangle to the blob
		circle(img_labeling, Point(x, y), 5, Scalar(255, 0, 0), 1);
		rectangle(img_labeling, Point(left, top), Point(left + width, top + height),
				Scalar(0, 0, 255), 1);

		//Convert integer to string for writing the idex of blobs
		std::ostringstream os;
		os << max_idx;
		//std::cout << os.str() << std::endl;
		putText(img_labeling, os.str(), Point(left + 20, top + 20), FONT_HERSHEY_SIMPLEX,	1, Scalar(255, 0, 0), 2);


		
		max_x = left;
		max_y = top;
		max_w = width;
		max_h = height;
		
		
		return img_labeling;
	}
	
	cv::Mat drawLabelingImage(Mat image)
	{
		Mat img_gray, img_labeling, img_binary;

		cvtColor(image, img_gray, COLOR_BGR2GRAY);	// Convert the image to Gray
		threshold(img_gray, img_binary, 127, 255, THRESH_BINARY);
		cvtColor(img_gray, img_labeling, COLOR_GRAY2BGR);


		Mat img_labels, stats, centroids;
		int numOfLables = connectedComponentsWithStats(img_binary, img_labels,
			stats, centroids, 8, CV_32S);


		//라벨링된 이미지중 특정 라벨을 컬러로 표현해주기  
		for (int y = 0; y<img_labels.rows; ++y) 
		{
			int *label = img_labels.ptr<int>(y);
			Vec3b* pixel = img_labeling.ptr<Vec3b>(y);

			for (int x = 0; x < img_labels.cols; ++x) 
			{
				if (label[x] == 3) 
				{
					pixel[x][2] = 0;
					pixel[x][1] = 255;
					pixel[x][0] = 0;
				}
			}
		}

		int max_idx, max_size = 0;
		//라벨링 된 이미지
		for (int j = 1; j < numOfLables; j++) 
		{
			int width = stats.at<int>(j, CC_STAT_WIDTH);
			int height = stats.at<int>(j, CC_STAT_HEIGHT);

			// Find the maximum rect
			if(width*height >= max_size)
			{
				max_idx = j;
				max_size = width*height;
			}
		}

		int area = stats.at<int>(max_idx, CC_STAT_AREA);
		int left = stats.at<int>(max_idx, CC_STAT_LEFT);
		int top = stats.at<int>(max_idx, CC_STAT_TOP);
		int width = stats.at<int>(max_idx, CC_STAT_WIDTH);
		int height = stats.at<int>(max_idx, CC_STAT_HEIGHT);

		int x = centroids.at<double>(max_idx, 0); //중심좌표
		int y = centroids.at<double>(max_idx, 1);
		
		// Draw circle on the center, rectangle to the blob
		circle(img_labeling, Point(x, y), 5, Scalar(255, 0, 0), 1);
		rectangle(img_labeling, Point(left, top), Point(left + width, top + height),
				Scalar(0, 0, 255), 1);

		//Convert integer to string for writing the idex of blobs
		std::ostringstream os;
		os << max_idx;
		//std::cout << os.str() << std::endl;
		putText(img_labeling, os.str(), Point(left + 20, top + 20), FONT_HERSHEY_SIMPLEX,	1, Scalar(255, 0, 0), 2);


		if( max_size > 0)
		{
			roi_x = left;
			roi_y = top;
			roi_w = width;
			roi_h = height;
		}
		
		
		return img_labeling;
	}
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


