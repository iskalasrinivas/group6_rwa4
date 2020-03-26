//
// Created by zeid on 2/28/20.
//

#ifndef GROUP6_RWA4_COMPETITION_H
#define GROUP6_RWA4_COMPETITION_H

#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "sensor.h"
#include "order_manager.h"

//class AriacOrderManager;

class Competition {

private:

	ros::NodeHandle comp_nh_;
	//AriacOrderManager order_manager_;
	AriacSensorManager sensor_;


	std::string competition_state_;
	double current_score_;
	ros::Publisher arm_1_joint_trajectory_publisher_;
	ros::Publisher arm_2_joint_trajectory_publisher_;

	sensor_msgs::JointState arm_1_current_joint_states_;
	sensor_msgs::JointState arm_2_current_joint_states_;
	bool arm_1_has_been_zeroed_;
	bool arm_2_has_been_zeroed_;
//	osrf_gear::Order order_;
	ros::Subscriber current_score_subscriber;

	// Subscribe to the '/ariac/competition_state' topic.
	ros::Subscriber competition_state_subscriber;

public:
	Competition();
	~Competition();
	/// Called when a new message is received.
	void current_score_callback(const std_msgs::Float32::ConstPtr & );

	/// Called when a new message is received.
	void competition_state_callback(const std_msgs::String::ConstPtr & );

	/// Called when a new JointState message is received.
	void arm_1_joint_state_callback(const sensor_msgs::JointState::ConstPtr & );

	void arm_2_joint_state_callback(const sensor_msgs::JointState::ConstPtr &);

	/// Create a JointTrajectory with all positions set to zero, and command the arm.
	void send_arm_to_zero_state(ros::Publisher &);

	void StartCompetition();

	void EndCompetition();

};


#endif //GROUP6_RWA4_COMPETITION_H
