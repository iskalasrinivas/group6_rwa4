/**
 * @file      include/sensor.h
 * @brief     Header file for Sensor
 * @author    Saurav Kumar
 * @author    Raja Srinivas
 * @author    Sanket Acharya
 * @author    Dinesh Kadirimangalam
 * @author    Preyash Parikh
 * @copyright 2020
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2020
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GROUP6_RWA4_SENSOR_H_
#define GROUP6_RWA4_SENSOR
#include <ariac_order_part.h>
#include <list>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>

#include <order_manager.h>


// class AriacOrderManager;

class AriacSensorManager {


private:
	std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> all_binParts;
	std::map<std::string, std::vector<geometry_msgs::Pose>> sorted_all_binParts;
	AriacOrderManager order_manager_ ;
	ros::NodeHandle sensor_nh_;

	ros::Subscriber camera_1_subscriber_;
	ros::Subscriber camera_4_subscriber_;
	ros::Subscriber camera_5_subscriber_;
	ros::Subscriber breakbeam_subscriber;
	ros::Subscriber quality_control_camera_subscriber_;

	geometry_msgs::TransformStamped transformStamped1_belt;
	geometry_msgs::TransformStamped transformStamped2_belt;
	geometry_msgs::TransformStamped transformStamped3_belt;
	tf2_ros::Buffer tf_buffer_belt;
	tf2_ros::TransformBroadcaster br_w_s_belt;
	tf2_ros::TransformBroadcaster br_s_c_belt;
	tf2_ros::TransformListener tf_listener_belt;

	geometry_msgs::TransformStamped transformStamped1_bin1;
	geometry_msgs::TransformStamped transformStamped2_bin1;
	geometry_msgs::TransformStamped transformStamped3_bin1;
    tf2_ros::Buffer tf_buffer_bin1;
    tf2_ros::TransformBroadcaster br_w_s_bin1;
	tf2_ros::TransformBroadcaster br_s_c_bin1;
	tf2_ros::TransformListener tf_listener_bin1;

	geometry_msgs::TransformStamped transformStamped1_bin2;
	geometry_msgs::TransformStamped transformStamped2_bin2;
	geometry_msgs::TransformStamped transformStamped3_bin2;
	tf2_ros::Buffer tf_buffer_bin2;
	tf2_ros::TransformBroadcaster br_w_s_bin2;
	tf2_ros::TransformBroadcaster br_s_c_bin2;
	tf2_ros::TransformListener tf_listener_bin2;

	osrf_gear::Model* tracking_part_;
	geometry_msgs::Pose tracking_pose_in_sensor;
	AriacOrderPart* faulty_part_;

	
	bool is_faulty;
	bool bin_part_faulty;

public:
	AriacSensorManager();
	~AriacSensorManager();

	void setPose(const geometry_msgs::Pose& , geometry_msgs::TransformStamped *);
	void setPose(const geometry_msgs::Pose&, geometry_msgs::Pose &);
	void setPose(geometry_msgs::TransformStamped*, geometry_msgs::Pose &);

	std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> getBinParts();

	
	void setTrackingPartInWorld();
	void beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void updateFaultyPartPose(AriacOrderPart*);
	void binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &, const std::string&);
	void binlogicalCameraCallback1(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void binlogicalCameraCallback2(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void setAllBinParts(const osrf_gear::LogicalCameraImage::ConstPtr &, std::string);
	void qualityControlSensorCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void agvLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void dropInAGV(const geometry_msgs::Pose &);
	void SortAllBinParts();
// void computeWorldTransformation(const geometry_msgs::Pose &, const geometry_msgs::Pose & , geometry_msgs::Pose & );
};

#endif // GROUP6_RWA4_SENSOR_H_
