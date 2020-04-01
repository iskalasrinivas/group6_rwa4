/**
 * @file      src/order_manager.cpp
 * @brief     Header file for building map
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


#include <osrf_gear/AGVControl.h>
#include <string>
#include <initializer_list>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/optional.hpp>
#include <order_manager.h>


//  AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(std::map<std::string, std::map<std::string, std::vector<geometry_msgs::Pose>>>* abp, std::map<std::string, std::vector<geometry_msgs::Pose>>* sabp):
arm1_{"arm1"}, all_binParts(abp), sorted_all_binParts(sabp),
isBinCameraCalled(false), part_is_faulty(false), task_pending(true), conveyor_parts_picked (false), order_segregated(false){
	ros::AsyncSpinner async_spinner(4);
	async_spinner.start();
	order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10,
			&AriacOrderManager::OrderCallback, this);
}

AriacOrderManager::~AriacOrderManager() {}


void AriacOrderManager::OrderCallback
(const osrf_gear::Order::ConstPtr& order_msg) {
	ROS_WARN(">>>>> OrderCallback");
	received_orders_.push_back(*order_msg);
	ROS_INFO_STREAM("no of orders "<< received_orders_.size() << std::endl);
	setOrderParts();
	while (!isBinCameraCalled) {
		ROS_INFO_STREAM("Bin camera is not called" << std::endl);
	}
	ROS_INFO_STREAM("Bin camera Called!! Starting segregating Orders." << std::endl);
	ros::Duration(1).sleep();
	segregateOrders();
}

void AriacOrderManager::setBinCameraCalled() {
	isBinCameraCalled = true;
}

bool AriacOrderManager::isSegregated(){
	return order_segregated;
}


void AriacOrderManager::setOrderParts() {
	ROS_INFO_STREAM("reading order." << std::endl);

	for (const auto &order : received_orders_) {
		auto order_id = order.order_id;
		auto shipments = order.shipments;
		for (const auto &shipment : shipments) {
			auto shipment_type = shipment.shipment_type;
			auto products = shipment.products;

			for (const auto &product : products) {
				std::string part_type = product.type;
				if (all_orderParts.count(part_type)) {
					AriacOrderPart* order_part = new AriacOrderPart(product.type, product.pose);

					all_orderParts[part_type].push_back(order_part);
				} else {
					AriacOrderPart* order_part = new AriacOrderPart(product.type, product.pose);
					std::vector<AriacOrderPart*> vec;
					vec.push_back(order_part);
					all_orderParts[part_type] = vec;
				}
			}
		}
	}
}



void AriacOrderManager::SubmitAGV(int num) {
	std::string s = std::to_string(num);
	ros::ServiceClient start_client =
			order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
	if (!start_client.exists()) {
		ROS_INFO("Waiting for the client to be ready...");
		start_client.waitForExistence();
		ROS_INFO("Service started.");
	}

	osrf_gear::AGVControl srv;
	// srv.request.kit_type = "order_0_kit_0";
	start_client.call(srv);

	if (!srv.response.success) {
		ROS_ERROR_STREAM("Service failed!");
	} else {
		ROS_INFO("Service succeeded.");
	}
}


ros::NodeHandle* AriacOrderManager::getnode() {
	return &order_manager_nh_;
}

void AriacOrderManager::setCurrentPose
(std::vector<AriacOrderPart*> &ariacOrderparts,
		const std::vector<geometry_msgs::Pose> &vecPose) {
	std::vector<geometry_msgs::Pose>::const_iterator
	it_vecPose = vecPose.begin();
	for (auto &orderPart : ariacOrderparts) {
		orderPart->setCurrentPose(*it_vecPose);
		it_vecPose++;
		ROS_WARN_STREAM( "order Type and Current Pose from bin"<<orderPart->getPartType() << " " << orderPart->getCurrentPose());
	}
}



void AriacOrderManager::segregateOrders() {

	for (const auto &orderPart : all_orderParts) {
		auto part_type = orderPart.first;
//		ROS_INFO_STREAM( "order Part type :"<< part_type);
		auto oVecPart =  orderPart.second;
		if(sorted_all_binParts->count(part_type)) {
			auto bin_vec = (*sorted_all_binParts)[part_type];
//			ROS_INFO_STREAM( "order PArt type"<< bin_vec.front());
			if(bin_vec.size() >= orderPart.second.size()) {
				auto opart_it = oVecPart.begin();
				auto bin_part = bin_vec.begin();

				for(opart_it = oVecPart.begin(), bin_part = bin_vec.begin(); opart_it != oVecPart.end();++opart_it, ++bin_part) {
					(*opart_it)->setCurrentPose(*bin_part);
//					ROS_WARN_STREAM( "order Type and Current Pose from bin"<<(*opart_it)->getPartType() << " " << (*opart_it)->getCurrentPose());
				}
//				setCurrentPose(oVecPart, (*sorted_all_binParts)[part_type]);
				bin_order_parts.insert({part_type, oVecPart});
			} else {
				ROS_INFO_STREAM("Sufficient Parts not available on bin lesser parts:have to be coded !!");
			}
		} else {
			conveyor_order_parts.insert({part_type, oVecPart});
		}
	}

	order_segregated = true;


	for (auto it1_part : bin_order_parts) {
		ROS_INFO_STREAM("Parts to be picked from Bin :  Type of Part " << it1_part.first << ", Num of Parts : "<< it1_part.second.size()<<std::endl);
	}
	for (auto it2_part : conveyor_order_parts) {
		ROS_INFO_STREAM("Parts to be picked from Belt : Type of Part " << it2_part.first << ", Num of Parts : "<< it2_part.second.size()<<std::endl);
	}
}

void AriacOrderManager::updateBinOrder(std::map<std::string, std::vector<AriacOrderPart*>>
		::iterator vec_it, std::vector<AriacOrderPart*>::iterator it){
	std::map<std::string, std::vector<AriacOrderPart*>> update_bin_order (vec_it, bin_order_parts.end());
	auto part_type = vec_it->first;
	std::vector<AriacOrderPart*> update_vector (it, update_bin_order[part_type].end());
	update_bin_order[part_type] = update_vector;
	bin_order_parts = update_bin_order;
}

std::map<std::string,
std::vector<AriacOrderPart*>> AriacOrderManager::getBinOrderParts() {
	return bin_order_parts;
}

std::map<std::string,
std::vector<AriacOrderPart*>> AriacOrderManager::getConveyorOrderParts() {
	return conveyor_order_parts;
}

void AriacOrderManager::removeConveyorPart(AriacOrderPart* orderPart) {
	auto part_type = orderPart->getPartType();
	if (conveyor_order_parts.size() != 0) {
		if(conveyor_order_parts.count(part_type)) {
			if (conveyor_order_parts[part_type].size() != 0) {
				conveyor_order_parts[part_type].pop_back();
			}
			if (conveyor_order_parts[part_type].size() == 0) {
				conveyor_order_parts.erase(part_type);
			}
		}
	}
}

void AriacOrderManager::removeBinPart(AriacOrderPart* orderPart) {
	auto part_type = orderPart->getPartType();
	if (bin_order_parts.size() != 0) {
		if (bin_order_parts[part_type].size() != 0) {
			bin_order_parts[part_type].pop_back();
			if (bin_order_parts[part_type].size() == 0) {
				bin_order_parts.erase(part_type);
			}
		}
	}
}

void AriacOrderManager::dropPartToAgv() {

}



bool AriacOrderManager::inVicinity(const geometry_msgs::Pose& world_part_pose) {
	double threshold_z = 0.1;
	double threshold_y = 0.35;
	return (arm1_.getHomeCartPose().position.z-
			world_part_pose.position.z < threshold_z &&
			arm1_.getHomeCartPose().position.y-
			world_part_pose.position.y < threshold_y);
}

void AriacOrderManager::transformAndPickPart
(const geometry_msgs::TransformStamped& world_msg, double y) {
	if (task_pending) {
		ROS_INFO("robot_controller_pathPlanning");
		geometry_msgs::Pose world_part_pose;
		world_part_pose.position.x = world_msg.transform.translation.x;
		world_part_pose.position.y = world_msg.transform.translation.y - y;
		world_part_pose.position.z = world_msg.transform.translation.z;
		world_part_pose.orientation.x = world_msg.transform.rotation.x;
		world_part_pose.orientation.y = world_msg.transform.rotation.y;
		world_part_pose.orientation.z = world_msg.transform.rotation.z;
		world_part_pose.orientation.w = world_msg.transform.rotation.w;


		pickPart(world_part_pose, y);
	}
}

void AriacOrderManager::pickPart(geometry_msgs::Pose world_part_pose, double y) {
	if (!arm1_.isPartAttached()) {
        world_part_pose.position.z += 0.02;
		world_part_pose.position.y -= y; 
		arm1_.GoToTarget(world_part_pose);
		if (inVicinity(world_part_pose)) {
			ROS_WARN_STREAM("Gripper toggled");
			arm1_.GripperToggle(true);
			while (!arm1_.isPartAttached()) {
				ROS_WARN_STREAM("Part not attached");
				world_part_pose.position.z += 0.004;
				world_part_pose.position.y -= 2*y;
				arm1_.GoToTarget(world_part_pose);
				world_part_pose.position.z -= 0.004;
				world_part_pose.position.y -= 2*y;
				arm1_.GoToTarget(world_part_pose);
			}
			ROS_INFO_STREAM("Part attached");

			world_part_pose.position.z += 0.2;
			world_part_pose.position.y -= y;
			arm1_.GoToTarget(world_part_pose);
		}
	} else {
		arm1_.GoToQualityCamera();
		task_pending = false;
	}
}

void AriacOrderManager::pickfromBin(const geometry_msgs::Pose& part_pose) {
	ROS_INFO_STREAM("Picking Part");
	ros::Duration(0.5).sleep();
	arm1_.GoToBinStaticPosition();
	ros::Duration(0.5).sleep();

	auto target_top_pose_1 = part_pose;
//	target_top_pose_1.position.y += 0.5;
	target_top_pose_1.position.z += 0.2;
	arm1_.GoToTarget(target_top_pose_1);
	ros::Duration(1.0).sleep();
//	auto target_top_pose_2 = target_top_pose_1;
////	target_top_pose_2.position.y -= 0.5;
//	arm1_.GoToTarget(target_top_pose_2);
//	ros::Duration(0.5).sleep();
	auto target_pose = part_pose;
	target_pose.position.z += 0.1;
	arm1_.GoToTarget(target_pose);
	arm1_.GripperToggle(true);
	if(!arm1_.isPartAttached()) {
		while (!arm1_.isPartAttached()) {
			target_pose.position.z -= 0.01;
			arm1_.GoToTarget(target_pose);
			ros::Duration(0.5).sleep();
		}
	}
//	arm1_.GoToTarget(target_top_pose_2);
	ros::Duration(0.5).sleep();
	arm1_.GoToTarget(target_top_pose_1);
	ros::Duration(0.5).sleep();
	arm1_.GoToBinStaticPosition();
	ros::Duration(0.5).sleep();

}

void AriacOrderManager::setConveyorPartsPicked(const bool & boolean) {
	conveyor_parts_picked = boolean;
}


bool AriacOrderManager::isConveyorPartsPicked() {
	return conveyor_parts_picked;
}

RobotController* AriacOrderManager::getArmObject(){
	return &arm1_;
}

//  void AriacOrderManager::pathplanning
//  (const geometry_msgs::TransformStamped& msg) {
//  segregateOrders();
//  while(conveyor_order_parts.size() != 0) {
//
//
//    pickPart(msg, 0.2);
//    std::string picked_part_id = identify_part();
//    removeConveyorPart(picked_part_id);
//    dropPartToAgv();
//    move_to_home_position();
//  }
//
//  while(bin_order_parts.size() != 0) {
//
//
//    pick_part_from_bin(msg);
//    std::string picked_part_id = identify_part();
//    removeBinPart(picked_part_id);
//    dropPartToAgv();
//    move_to_home_position();
//
//  }
//}
//
//void AriacOrderManager::segregateOrders() {
//	for (auto it_part : all_orderParts) {
//		ROS_INFO_STREAM("segregate Orders Called ! Type of Part " << it_part.first << " Num of Parts "<< it_part.second.size()<<std::endl);
//	}
//	for (auto cam_it : *all_binParts) {
//		for (auto part_it : cam_it.second) {
//			ROS_INFO_STREAM("camera " << cam_it.first << " Part type "<< part_it.first<< " "<< part_it.second.size()<<std::endl);
//		}
//	}
//	for (const auto &orderPart : all_orderParts) {
//		for (const auto &binPart : *all_binParts) {
//			auto oVecPart = orderPart.second;
//			auto oType = orderPart.first;
//			auto binMapPart = binPart.second;
//			for (auto bPart : binMapPart) {
//				if (oType == bPart.first && bPart.second.size() > oVecPart.size()) {
//					setCurrentPose(oVecPart, bPart.second);
//					bin_order_parts.insert({oType, oVecPart});
//					// } else if(oType ==
//					// bPart.first && bPart.second.size() < oVecPart.size()){
//					// setCurrentPose(oVecPart, bPart.second);
//					// bin_order_parts.insert({oType, oVecPart});
//					// conveyor_order_parts.insert({oType, oVecPart});
//				} else {
//					conveyor_order_parts.insert({oType, oVecPart});
//				}
//			}
//		}
//	}
//	for (auto it1_part : bin_order_parts) {
//		ROS_INFO_STREAM("From Bin :  Type of Part " << it1_part.first << " Num of Parts "<< it1_part.second.size()<<std::endl);
//	}
//	for (auto it2_part : conveyor_order_parts) {
//		ROS_INFO_STREAM("From Belt : Type of Part " << it2_part.first << " Num of Parts "<< it2_part.second.size()<<std::endl);
//	}
//}
