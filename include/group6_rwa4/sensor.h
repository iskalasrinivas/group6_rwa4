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

#ifndef GROUP6_RWA4_SENSOR
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
#include "order_manager.h"


// class AriacOrderManager;

class AriacSensorManager {


private:
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> all_binParts;
	AriacOrderManager order_manager_ ;
	ros::NodeHandle sensor_nh_;

	ros::Subscriber camera_1_subscriber_;
	ros::Subscriber camera_4_subscriber_;
	ros::Subscriber camera_5_subscriber_;
	ros::Subscriber breakbeam_subscriber;
    ros::Subscriber quality_control_camera_subscriber_;

	geometry_msgs::TransformStamped transformStamped1;
	geometry_msgs::TransformStamped transformStamped2;
	geometry_msgs::TransformStamped transformStamped3;
	tf2_ros::Buffer tfBuffer;



	tf2_ros::TransformBroadcaster br_w_s;
	tf2_ros::TransformBroadcaster br_s_c;
	bool object_detected = false;
	bool conveyor_parts_picked = false;

public:
	AriacSensorManager();
	~AriacSensorManager();
	void setPose(const geometry_msgs::Pose pose, geometry_msgs::TransformStamped &);
	void setPose(const geometry_msgs::Pose pose, geometry_msgs::Pose &);
    void setPose(const geometry_msgs::TransformStamped transformStamped, geometry_msgs::Pose &pose);
    std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>> getBinParts();
    void computeWorldTransformation(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &, std::string);
	void binlogicalCameraCallback1(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void binlogicalCameraCallback2(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void setAllBinParts(const osrf_gear::LogicalCameraImage::ConstPtr &, std::string);
	void breakBeamCallback(const osrf_gear::Proximity::ConstPtr &);
	bool isObjectDetected();
	void qualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);

};

#endif //GROUP6_RWA4_SENSOR
