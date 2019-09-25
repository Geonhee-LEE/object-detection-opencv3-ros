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

#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
 
 
#include "std_msgs/Float32MultiArray.h"

#define SSTR( x ) static_cast< std::ostringstream & >( ( std::ostringstream() << std::dec << x ) ).str()

#define MINIMUM_LABEL_SIZE 15000
#define MAXIMUM_LABEL_SIZE 30000

using namespace cv;  
using namespace std;  



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
  ros::Publisher obj_center_pub_;
  
  // img proc
  cv::Mat img_proc(cv::Mat);
  cv::Mat raw_img;
  cv::Mat output_img;
  cv::Mat roi_img;
  cv::Mat object_img;

  int obj_label_x, obj_label_y, obj_label_w, obj_label_h;
  int mean_cnt;
  int roi_x, roi_y, roi_w, roi_h;
  bool getROI_flg;
	
  // For tracking
  bool track_init;
  uint8_t fail_cnt;
  Ptr<Tracker> tracker;
  Rect2d bbox;
  uint16_t tracked_size;
	
	// Window position
	uint8_t move_win_x, move_win_y; 
			
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
			
		obj_center_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/object/center_pos", 100);

		output_img=cv::Mat::zeros(480,640,CV_8UC3);	
		obj_label_x, obj_label_y, obj_label_w, obj_label_h = 0, 0, 0, 0;
		roi_x = 0, roi_y = 0, roi_w = 0, roi_h = 0;	
		mean_cnt = 0;
		getROI_flg = false;
		track_init = true;
		fail_cnt = 0;
		tracked_size = 0;
		move_win_x = 150, move_win_y = 150;
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
			cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(640, 480),0,0,CV_INTER_NN);	  
			raw_img = cv_ptr->image;
			//Show ROI image about object(enclosure)
			if( roi_x + roi_w <= cv_ptr->image.size().width && roi_y + roi_h <= cv_ptr->image.size().height )
			{
				roi_x = 0;
				roi_y = 70;
				roi_w = 640;
				roi_h = 150;
				cv_ptr->image = setROI(cv_ptr->image, roi_x, roi_y, roi_w, roi_h);
			}
			else 
				ROS_INFO_STREAM( "NOT ROI: "<< roi_x << ", " << roi_y << ", " << roi_w <<  ", " << roi_h);	 
				
			cv::waitKey(3);  
					
  		Mat src, hist_equal_dst;
			/// Convert to grayscale
			cvtColor(cv_ptr->image, src, CV_BGR2GRAY );

			/// Apply Histogram Equalization
			equalizeHist(src, hist_equal_dst);

			namedWindow("Raw Image", WINDOW_AUTOSIZE);				// Create a window for display
			imshow("Raw Labeling Image", hist_equal_dst);			// Show our image inside it
			moveWindow("Raw Labeling Image", move_win_x + 640, move_win_y);

			// Send the raw image for processing second HSV node.
			roi_image_pub_.publish(cv_ptr->toImageMsg());  
			getROI_flg = false;
  }
	
  // Object tracking function using KF
  void tracking_object(cv::Mat _roi_img, cv::Mat obj_img, std::string trackerType)
  {	  
		if(track_init == true)
		{
			ROS_INFO_STREAM( "Init tracking");	
									// List of tracker types in OpenCV 3.4.1
			#if (CV_MINOR_VERSION < 3)
			{
				tracker = Tracker::create(trackerType);
			}
			#else
			{
				if (trackerType == "BOOSTING")
					tracker = TrackerBoosting::create();
				if (trackerType == "MIL")
					tracker = TrackerMIL::create();
				if (trackerType == "KCF")
					tracker = TrackerKCF::create();
				if (trackerType == "TLD")
					tracker = TrackerTLD::create();
				if (trackerType == "MEDIANFLOW")
					tracker = TrackerMedianFlow::create();
				if (trackerType == "GOTURN")
					tracker = TrackerGOTURN::create();
				if (trackerType == "MOSSE")
					tracker = TrackerMOSSE::create();
			}
			#endif     

			// Define initial bounding box and set the labeled image as reference image  using labeling and hsv 	
			bbox.x = obj_label_x; // = selectROI(roi_img);
			bbox.y = obj_label_y; // = selectROI(roi_img);
			bbox.width = obj_label_w; // = selectROI(roi_img);
			bbox.height = obj_label_h; // = selectROI(roi_img);

			// Uncomment the line below to select a different bounding box 
			// bbox = selectROI(roi_img, false); 
			// Display bounding box. 
			rectangle(_roi_img, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
			
			imshow("Tracking", _roi_img); 
			moveWindow("Tracking", move_win_x, move_win_x +200);
			tracker->init(_roi_img, bbox);
			
			// Fail to fine the labeling
			if(bbox.x == 0 && bbox.y == 0  && bbox.width == _roi_img.size().width && bbox.height == _roi_img.size().height)
			{
				destroyWindow("Bbox");
				return ;					
			}

			imshow("Bbox", object_img); 
			moveWindow("Bbox", move_win_x, move_win_y + 450);
			track_init = false;
		}
     
		ROS_INFO_STREAM( "Tracking");
    
		// Start timer
    double timer = (double)getTickCount();
         
    // Update the tracking result
    bool ok = tracker->update(_roi_img, bbox);
         
    // Calculate Frames per second (FPS)
    float fps = getTickFrequency() / ((double)getTickCount() - timer);
    float ang = static_cast<float>(bbox.height/bbox.width);
       
    if(ok)
    {
      // Tracking success : Draw the tracked object
			fail_cnt = 0;
			tracked_size = bbox.width * bbox.height;
									rectangle(_roi_img, bbox, Scalar( 255, 0, 0 ), 2, 1 );
			ROS_INFO_STREAM( "bbox: "<< bbox.x << ", " << bbox.y << ", " << bbox.width <<  ", " << bbox.height);	
			// Draw circle on the center, rectangle to the blob
			circle(_roi_img, Point(bbox.x + bbox.width * 0.5, bbox.y +  bbox.height * 0.5), 5, Scalar(255, 255, 0), 3);	
											
			//Send the object center position through ROS topic 
			std_msgs::Float32MultiArray msg_array;
			msg_array.data.push_back(bbox.x + bbox.width * 0.5);	//cur_x
			msg_array.data.push_back(bbox.y +  bbox.height * 0.5+ roi_y); //cur_y		
			obj_center_pub_.publish(msg_array);
			}
			else
			{		
				// Tracking failure detected.
				fail_cnt++;
				if(fail_cnt > 5)
				{
					track_init = true;		
					putText(_roi_img, "Tracking failure detected ", Point(300,130), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);	
				}		
			}
    
		// Offset point since shadow
		circle(_roi_img, Point(bbox.x + bbox.width * 0.5, bbox.y+ bbox.height - 40), 3, Scalar(0,0,255), 3);			 

    // Display tracker type on frame
    putText(_roi_img, trackerType + " Tracker", Point(500,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(50,170,50),1);
         
    // Display FPS on frame
    putText(_roi_img, "FPS : " + SSTR(int(fps)), Point(500,50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(50,170,50), 1);
    // Display angle on frame
    putText(_roi_img, "Ang : " + SSTR(float(ang)), Point(500,80), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(50,170,50), 1);
    // Display frame.
    imshow("Tracking", _roi_img); 
  }

  // Callback function about target img for img proc(enclosure)
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
			
		// Object center point
		// Filtering on the Conveyer
		cv::medianBlur(cv_ptr->image, cv_ptr->image, 3);
		namedWindow("Object Labeling Image", WINDOW_AUTOSIZE);				// Create a window for display
		imshow("Object Labeling Image", getLabeledObjectImage(cv_ptr->image));			// Show our image inside it
		moveWindow("Object Labeling Image", move_win_x, move_win_y);
		
		// enclosure size: ( Object label: 338, 81)
		//ROS_INFO_STREAM( "Object label: "<< max_w << ", " << max_h);	
		mean_cnt++;
			
		if(mean_cnt == 5)
		{		
			// Kalman filter tracking
			if(obj_label_x + obj_label_w <= cv_ptr->image.size().width && obj_label_y + obj_label_h <= cv_ptr->image.size().height )
			{
				// object image
				object_img = setROI(raw_img, roi_x + obj_label_x , roi_y + obj_label_y,  obj_label_w, obj_label_h);
				
				// ROI image 
				roi_img = setROI(raw_img, roi_x  , roi_y ,  roi_w, roi_h);
				
				//Tracking function, find the object_img from roi_img
				tracking_object(roi_img, object_img, "KCF");
			}
			mean_cnt = 0;
		}	  
		cv::waitKey(3);  
		result_image_pub_.publish(cv_ptr->toImageMsg());
  }
 
  // Callback function about back ground img(conveyer)
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
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(640, 480),0,0,CV_INTER_NN);
		 
    namedWindow("Conveyer Labeling Image", WINDOW_AUTOSIZE);				// Create a window for display
    imshow("Conveyer Labeling Image",cv_ptr->image);						// Show our image inside it
    getROI_flg = true;
    cv::waitKey(3);
  }


  cv::Mat max_labeling_bg(cv::Mat src_mat)
  {
		cv::Mat out_img;
		src_mat = adjustFilter(src_mat);

		//Labeling of maximum blob,
		out_img = bgLabelingImage(src_mat);

		return out_img;
  }

  // Adjust filters
  cv::Mat adjustFilter(Mat src_img)
	{		
		// 필터 효과를 더 두드러지게 5x5 구조 요소를 사용
		cv::Mat element3(3, 3, CV_8U, cv::Scalar(1));
		cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
		cv::Mat return_img;				

		// 영상 닫힘과 영상 열림
		cv::morphologyEx(src_img,return_img,cv::MORPH_CLOSE,element3);
		cv::morphologyEx(return_img,return_img,cv::MORPH_OPEN,element3);
		cv::dilate(return_img,return_img,element5);
		//cv::namedWindow("Closed Image");
		//cv::imshow("Closed Image", return_img);
		
		return return_img;
	}
	
  // Set ROI
  cv::Mat setROI(Mat src_img, int X, int Y, int W, int H)
	{		
		// 관심영역 설정 (set ROI (X, Y, Width, Height)).
		Rect rect(X, Y, W, H);

		// 관심영역 자르기 (Crop ROI).
		Mat img_roi = src_img(rect);

		// show
		//imshow("raw_image", img_roi);
		
		return img_roi;
	}
		
  // Get the labeling of object(enclosure)
  cv::Mat getLabeledObjectImage(Mat image)
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
			if(width*height >= max_size && width*height > MINIMUM_LABEL_SIZE && width*height < MAXIMUM_LABEL_SIZE )
			{
				max_idx = j;
				max_size = width*height;
				
				// If the reference_img for tracking is lower than labling img + offset for preventing osillation, then change the reference img 
				uint16_t offset = 2000;
				if(tracked_size + offset < max_size )
					track_init = true;
					
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
		
		obj_label_x = left;
		obj_label_y = top;
		obj_label_w = width;
		obj_label_h = height;		
		
		return img_labeling;
	}
	
  // Get the lebeling of background(conveyer)
  cv::Mat bgLabelingImage(Mat image)
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

		/* Foung the conveyer position
		if( max_size > 0)
		{
			roi_x = left;
			roi_y = top;
			roi_w = width;
			roi_h = height;
		}*/
		
		
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


