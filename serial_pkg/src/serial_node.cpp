/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <dynamic_reconfigure_test/Comm.h>
#include <dynamic_reconfigure_test/dyn_reconfig_testConfig.h>
#include <opencv_pkg/ImgData.h>

serial::Serial ser;
std_msgs::String comm_data; 
std_msgs::String img_comm_data; 
std_msgs::String result;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

bool imgdata_callback(opencv_pkg::ImgData::Request &req, opencv_pkg::ImgData::Response &res)
{
	img_comm_data.data=req.imgdata;
	ROS_INFO_STREAM("img data callback : "<< img_comm_data.data ); 
	return true;
}
bool service_callback(dynamic_reconfigure_test::Comm::Request &req, dynamic_reconfigure_test::Comm::Response &res)
{
	comm_data.data=req.senddata;
	ROS_INFO_STREAM("comm_data : " <<  comm_data.data); 
	return true;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
	ros::ServiceServer service;
	ros::ServiceServer imgdata_service;
	service = nh.advertiseService("comm_topic", service_callback);
	imgdata_service= n.advertiseService("imgdata_topic", imgdata_callback);

    try
    {
        ser.setPort("/dev/ttySAC0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        //return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    ros::Rate loop_rate(1000);



    while(ros::ok()){

       ros::spinOnce();
	
	ser.write(comm_data.data);


        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
		
		comm_data.data="";
		result.data="";

        }
        loop_rate.sleep();

    }

	ros::spin();

}

