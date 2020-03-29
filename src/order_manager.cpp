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
AriacOrderManager::AriacOrderManager(std::map<std::string, std::map<std::string,
		std::vector<geometry_msgs::Pose>>>* abp): arm1_{"arm1"}, all_bin_parts(abp),
		isBinCameraCalled(false), part_is_faulty(false), task_pending(true), conveyor_parts_picked (false) {
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
				ROS_INFO_STREAM("bin camera is not called" << std::endl);
			}
			segregateOrders();
		}

		void AriacOrderManager::setBinCameraCalled() {
			isBinCameraCalled = true;
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
							auto vector_by_parts = all_orderParts[part_type];
							AriacOrderPart order_part(product.type, product.pose);
							vector_by_parts.push_back(order_part);
							all_orderParts.insert({part_type, vector_by_parts});
						} else {
							AriacOrderPart order_part(product.type, product.pose);
							std::vector<AriacOrderPart> vec;
							vec.push_back(order_part);
							all_orderParts.insert({part_type, vec});
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
		(std::vector<AriacOrderPart> &ariacOrderparts,
				const std::vector<geometry_msgs::Pose> &vecPose) {
			std::vector<geometry_msgs::Pose>::const_iterator
			it_vecPose = vecPose.begin();
			for (auto &orderPart : ariacOrderparts) {
				orderPart.setCurrentPose(*it_vecPose);
				++it_vecPose;
			}
		}

		void AriacOrderManager::segregateOrders() {
			for (const auto &orderPart : all_orderParts) {
				for (const auto &binPart : *all_bin_parts) {
					auto oVecPart = orderPart.second;
					auto oType = orderPart.first;
					auto binMapPart = binPart.second;
					for (auto bPart : binMapPart) {
						if (oType == bPart.first && bPart.second.size() > oVecPart.size()) {
							setCurrentPose(oVecPart, bPart.second);
							bin_order_parts.insert({oType, oVecPart});
							// } else if(oType ==
							// bPart.first && bPart.second.size() < oVecPart.size()){
							// setCurrentPose(oVecPart, bPart.second);
							// bin_order_parts.insert({oType, oVecPart});
							// conveyor_order_parts.insert({oType, oVecPart});
						} else {
							conveyor_order_parts.insert({oType, oVecPart});
						}
					}
				}
			}
		}
    
    void AriacOrderManager::updateBinOrder(std::map<std::string, std::vector<AriacOrderPart>> 
                                           ::iterator vec_it, std::vector<AriacOrderPart>::iterator it){                            
         std::map<std::string, std::vector<AriacOrderPart>> update_bin_order (vec_it, bin_order_parts.end());
         auto part_type = vec_it->first;
         std::vector<AriacOrderPart> update_vector (it, update_bin_order[part_type].end());
         update_bin_order[part_type] = update_vector;
         bin_order_parts = update_bin_order;
    }

		std::map<std::string,
		std::vector<AriacOrderPart>> AriacOrderManager::getBinOrderParts() {
			return bin_order_parts;
		}

		std::map<std::string,
		std::vector<AriacOrderPart>> AriacOrderManager::getConveyorOrderParts() {
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

		void AriacOrderManager::moveToTarget(geometry_msgs::Pose final_pose) {
			std::vector<geometry_msgs::Pose> waypoints;
			geometry_msgs::Pose dt_pose;

			dt_pose.position.x =
					(final_pose.position.x - arm1_.getHomeCartPose().position.x) / 10;
			dt_pose.position.y =
					(final_pose.position.y - arm1_.getHomeCartPose().position.y) / 10;
			dt_pose.position.z =
					(final_pose.position.z - arm1_.getHomeCartPose().position.z) / 10;
			dt_pose.orientation.x = 0;
			dt_pose.orientation.y = 0;
			dt_pose.orientation.z = 0;
			dt_pose.orientation.w = 0;

			geometry_msgs::Pose current_pose;
			for (int i = 1; i <= 10; i++) {
				current_pose.position.x = arm1_.getHomeCartPose().position.x +
						i * dt_pose.position.x;
				current_pose.position.y = arm1_.getHomeCartPose().position.y +
						i * dt_pose.position.y;
				current_pose.position.z = arm1_.getHomeCartPose().position.z +
						i * dt_pose.position.z;
				current_pose.orientation.x = arm1_.getHomeCartPose().orientation.x +
						i * dt_pose.orientation.x;
				current_pose.orientation.y = arm1_.getHomeCartPose().orientation.y +
						i * dt_pose.orientation.y;
				current_pose.orientation.z = arm1_.getHomeCartPose().orientation.z +
						i * dt_pose.orientation.z;
				current_pose.orientation.w = arm1_.getHomeCartPose().orientation.w +
						i * dt_pose.orientation.w;
				waypoints.emplace_back(current_pose);
			}
			arm1_.GoToTarget(waypoints);
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
		(const geometry_msgs::TransformStamped& world_msg, int y) {
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

		void AriacOrderManager::pickPart(geometry_msgs::Pose world_part_pose, int y) {
			if (!arm1_.isPartAttached()) {
				//  ROS_INFO("part not attached");
				//  ROS_INFO_STREAM(msg.transform.translation.x
				//  <<","<< msg.transform.translation.y<<","<< msg.transform.translation.z);
				arm1_.GoToTarget(world_part_pose);
				//    ROS_INFO("going toward part");
				//  ROS_INFO_STREAM("gap: "<<
				//  arm1_.getHomeCartPose().position.z- world_part_pose.position.z << ","<<
				// arm1_.getHomeCartPose().position.y- world_part_pose.position.y);
				if (inVicinity(world_part_pose)) {
					//      arm1_.GoToTarget(world_part_pose);
					// arm1_.PickPart(world_part_pose);
					arm1_.GripperToggle(true);
					world_part_pose.position.z += 0.2;
					world_part_pose.position.y -= y;   // 0.5
					arm1_.GoToTarget(world_part_pose);
				}
			} else {
				arm1_.GoToQualityCamera();
				task_pending = false;
			}
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
