#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <iostream>
#include "order_manager.h"
//#include "../include/group6_rwa4/competition.h"
#include "competition.h"
//#include "../include/group6_rwa4/sensor.h"


int main(int argc, char **argv) {

	ROS_INFO("Starting main function");
	ros::init(argc, argv, "ariac_manager_node");
	ros::AsyncSpinner async_spinner(4);
	async_spinner.start();

//
//	Competition mycompetition;
//
//	while(ros::ok()){
//		ROS_INFO_STREAM("HELLO");
//		std::cout << "OK";
//		ros::waitForShutdown();
//	}


	return 0;
}
