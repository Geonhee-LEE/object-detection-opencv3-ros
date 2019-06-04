#include <dynamic_reconfigure_test/hsvInts.h>
#include <dynamic_reconfigure_test/dyn_reconfig_testConfig.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv_pkg/ImgData.h>
#include <rnd_ws/labeling_geon.h>
#include "BlobLabeling.cpp"
#include "labeling_geon.cpp"
#include <math.h>
#include <iostream>
#include <sstream>

dynamic_reconfigure_test::hsvInts::Request g_req;
ros::ServiceClient imgdata_client;
opencv_pkg::ImgData srv;
std_msgs::String senddata2dsp;
CLabeling label;
static const std::string OPENCV_WINDOW = "IMAGE WINDOW";
cv::Mat hsv_conversion(cv::Mat);
void transmission_data(std_msgs::String);
void robot_movement(int,int);
void neck_movement(int, int);
cv::Mat img_proc(cv::Mat);
cv::Mat output_img;


CLabeling::CLabeling(void)
{
};

CLabeling::~CLabeling(void)
{
};
cv::Mat img_proc(cv::Mat src_mat)
{
	cv::Mat dst_mat(cv::Size(src_mat.size()), src_mat.type());

	dst_mat=hsv_conversion(src_mat);

	if(label.CLabeling::set_Label(dst_mat,100))
	{
		if(label.set_MaxLabel())
		{
			label.draw_MaxLabel(output_img,cv::Scalar(255,255,0),1);
			neck_movement(label.get_MaxLabel_FirstPt().x + label.get_MaxLabel_width()* 0.5, label.get_MaxLabel_FirstPt().y + label.get_MaxLabel_height()*0.5);
		}
	}
	cv::imshow(OPENCV_WINDOW,output_img);
	//transmission_data(command);
	
	return dst_mat;	
}
void neck_movement(int x, int y)
{
	std::ostringstream os;
	int rl_neck,ud_neck;
	int camera_ud_resolution=30;
	int camera_rl_resolution=50;
	int motor_deg;

	rl_neck=150-(double)((x-160)*camera_rl_resolution*0.00625);//160 denum
	ud_neck=150+(double)((y-120)*camera_ud_resolution*0.00833);//120 denum

	ROS_INFO("rl neck %d ud_neck %d" , rl_neck,ud_neck);
	
	if(abs(rl_neck)>abs(ud_neck))
	{
		motor_deg= (66.6666)*(double)(rl_neck)+20000;
		os<<motor_deg;
		//std::cout <<os.str() <<Std::endl;
		senddata2dsp.data= "PS00023," +os.str()+ ";";
		ROS_INFO_STREAM("senddata " << senddata2dsp.data);
	}
	else
	{
                motor_deg= (66.6666)*(double)(ud_neck)+20000;
                os<<motor_deg;
                //std::cout <<os.str() <<Std::endl;
                senddata2dsp.data= "PS00024," +os.str()+ ";";
                ROS_INFO_STREAM("senddata " << senddata2dsp.data);

	}
}
void robot_movement(int x, int y)
{
}
void transmission_data( std_msgs::String data)
{
	srv.request.imgdata=data.data;
	if(imgdata_client.call(srv))
	{
	}
	else
	{
		 ROS_ERROR("imgdata_Fail");
	}
}
cv::Mat hsv_conversion(cv::Mat src_Mat)
{
	cv::Mat hsv_img(cv::Size(320,240), src_Mat.type());

	cv::Scalar hsv_min =cvScalar((int)(g_req.h_min),(int)(g_req.s_min),(int)(g_req.v_min));

	cv::Scalar hsv_max =cvScalar((int)(g_req.h_max),(int)(g_req.s_max),(int)(g_req.v_max));
	
	cv::cvtColor(src_Mat,hsv_img,CV_RGB2HSV);
	cv::inRange(hsv_img, hsv_min, hsv_max,hsv_img);
	cv::Mat element10(10,10,CV_8U,cv::Scalar(1));
	cv::morphologyEx(hsv_img,hsv_img,cv::MORPH_CLOSE,element10);
	cv::imshow("hsv_img",hsv_img);
	
	return hsv_img;	
}
class ImageConverter
{
  ros::NodeHandle nh_;
  //ServiceClient sc;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
ros::ServiceServer service;
 dynamic_reconfigure_test::hsvInts srv;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    service = nh_.advertiseService("hsv_topic",&ImageConverter::hsv,this);
 imgdata_client=nh_.serviceClient<opencv_pkg::ImgData>("imgdata_topic");

    cv::namedWindow(OPENCV_WINDOW);
	output_img=cv::Mat::zeros(240,320,CV_8UC3);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
 
 bool hsv(dynamic_reconfigure_test::hsvInts::Request &req, dynamic_reconfigure_test::hsvInts::Response &res)
 {  
	g_req.h_min=req.h_min;
	g_req.h_max=req.h_max;
	g_req.s_min=req.s_min;
	g_req.s_max=req.s_max;
	g_req.v_min=req.v_min;
	g_req.v_max=req.v_max;
	ROS_INFO("OPENCV_CALLBACK");
     	return true;
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
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


