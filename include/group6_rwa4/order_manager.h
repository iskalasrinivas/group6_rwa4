/**
 * @file      include/order_manager.h
 * @brief     Header file for Order manager
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
#ifndef GROUP6_RWA4_ORDER_MANAGER_H_
#define GROUP6_RWA4_ORDER_MANAGER_H_

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <robot_controller.h>
#include <ariac_order_part.h>


using std::vector;

class AriacOrderManager {
private:
	ros::NodeHandle order_manager_nh_;
	ros::Subscriber order_subscriber_;
	std::vector<osrf_gear::Order> received_orders_;
	vector<AriacOrderPart*> part_manager;
	std::vector<std::string> product_type;
	RobotController arm1_;
	tf::TransformListener part_tf_listener_;
	std::pair<std::string, geometry_msgs::Pose> product_type_pose_;
	std::string object;
	std::map<std::string, std::vector<std::string>> product_frame_list_;
	osrf_gear::Order order_;

	std::map<std::string,
	std::map<std::string, std::vector<geometry_msgs::Pose>>>* all_binParts;
	std::map<std::string, std::vector<geometry_msgs::Pose>>* sorted_all_binParts;

	std::map<std::string, std::vector<AriacOrderPart*>> all_orderParts;
	std::map<std::string, std::vector<AriacOrderPart*>> conveyor_order_parts;
	std::map<std::string, std::vector<AriacOrderPart*>> bin_order_parts;
	bool task_pending;
	bool isBinCameraCalled;
	bool part_is_faulty;
	bool conveyor_parts_picked;
	bool order_segregated;
	geometry_msgs::Pose quality_control_camera_pose;
	geometry_msgs::Pose faulty_bin_pose;
	geometry_msgs::Pose agv_pose;

public:
	explicit AriacOrderManager(std::map<std::string,
			std::map<std::string, std::vector<geometry_msgs::Pose>>>*,
			std::map<std::string, std::vector<geometry_msgs::Pose>>*);
	~AriacOrderManager();
	void OrderCallback(const osrf_gear::Order::ConstPtr&);
	void setOrderParts();
	void setCurrentPose(std::vector<AriacOrderPart*> &,
			const std::vector<geometry_msgs::Pose> &);
	void segregateOrders();
	std::map<std::string, std::vector<AriacOrderPart*>> getBinOrderParts();
	std::map<std::string, std::vector<AriacOrderPart*>> getConveyorOrderParts();
	void removeConveyorPart(AriacOrderPart*);
	void removeBinPart(AriacOrderPart* orderPart);
	void dropPartToAgv();
	void SubmitAGV(int);
	ros::NodeHandle* getnode();
	void setBinCameraCalled();
	void updateBinOrder(std::map<std::string, std::vector<AriacOrderPart*>>::iterator, std::vector<AriacOrderPart*>::iterator);
	bool inVicinity(const geometry_msgs::Pose&);
	void transformAndPickPart(const geometry_msgs::TransformStamped& , double);
	void pickPart(geometry_msgs::Pose, double);
	void setConveyorPartsPicked(const bool & );
	bool isConveyorPartsPicked();
	bool isSegregated();
	void pickfromBin(const geometry_msgs::Pose&);
	RobotController* getArmObject();
};

#endif //  GROUP6_RWA4_ORDER_MANAGER_H_


