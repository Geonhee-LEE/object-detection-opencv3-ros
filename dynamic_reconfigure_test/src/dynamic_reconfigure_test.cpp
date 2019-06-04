#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_test/dyn_reconfig_testConfig.h>
#include "std_msgs/String.h"
#include <sstream>
#include "dynamic_reconfigure_test/hsvInts.h"
#include "dynamic_reconfigure_test/Comm.h"

ros::ServiceClient client;
ros::ServiceClient comm_client;
dynamic_reconfigure_test::hsvInts srv;
dynamic_reconfigure_test::Comm comm_srv;

/*ros::ServiceServer service;

bool hsv(dynamic_reconfigure_test::hsvInts::Request &req, dynamic_reconfigure_test::hsvInts::Response &res)
{
	req.h_min=h_min_value;
	ROS_INFO("%d", (int)req.h_min);
	return true;
}*/
void callback(dynamic_reconfigure_test::dyn_reconfig_testConfig &config, uint32_t level) {
	ros::NodeHandle nh_;
	client=nh_.serviceClient<dynamic_reconfigure_test::hsvInts>("hsv_topic");
	comm_client = nh_.serviceClient<dynamic_reconfigure_test::Comm>("comm_topic");

        srv.request.h_min=config.H_min;
	srv.request.h_max=config.H_max;
	srv.request.s_min=config.S_min;
	srv.request.s_max=config.S_max;
	srv.request.v_min=config.V_min;
	srv.request.v_max=config.V_max;

	//srv.request.receivedata=config.ReceiveData;
	comm_srv.request.senddata=config.SendData;

	 if(client.call(srv))
	{
	}else{
                ROS_ERROR("hsv_Fail");
        }

	if(comm_client.call(comm_srv))
        {
        }
        else{
                ROS_ERROR("comm_Fail");
        }
	//ROS_INFO("H_min: %d, H_max: %d, S_min: %d, S_max: %d, V_min: %d, V_max: %d", config.H_min, config.H_max, config.S_min, config.S_max, config.V_min, config.V_max);
}

int main(int argc, char **argv) {
	

      ros::init(argc, argv, "dynamic_reconfigure_test");
      
      dynamic_reconfigure::Server<dynamic_reconfigure_test::dyn_reconfig_testConfig> server;
      dynamic_reconfigure::Server<dynamic_reconfigure_test::dyn_reconfig_testConfig>::CallbackType f;



	//ros::Publisher hsv_pub = n.advertise<std_msgs::String>("hsv_chatter",1000);
      f = boost::bind(&callback, _1, _2);
      server.setCallback(f);
      ros::spin();
      return 0;
}
