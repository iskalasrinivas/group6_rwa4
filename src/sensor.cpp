/**
 * @file      src/sensor.cpp
 * @brief     Source file for Sensor
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

#include <sensor.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


AriacSensorManager::AriacSensorManager() : order_manager_(& all_binParts, &sorted_all_binParts), is_faulty(false),task_completed(false), in_process(false), tf_listener_belt(tf_buffer_belt),
tf_listener_bin1(tf_buffer_bin1), tf_listener_bin2(tf_buffer_bin2){
	ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
	ros::AsyncSpinner async_spinner(4);
	async_spinner.start();
	camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10 ,
			&AriacSensorManager::binlogicalCameraCallback1, this);
	camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10 ,
			&AriacSensorManager::beltlogicalCameraCallback, this);
	camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10 ,
			&AriacSensorManager::binlogicalCameraCallback2, this);
	quality_control_camera_subscriber_ =
			sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10 ,
					&AriacSensorManager::qualityControlSensorCallback, this);
	tracking_part_ = nullptr;
	faulty_part_ = nullptr;
	bin_part_faulty = false;
}

AriacSensorManager::~AriacSensorManager() {
}

void AriacSensorManager::setPose
(const geometry_msgs::Pose & pose,
		geometry_msgs::TransformStamped * transformStamped ) {
	transformStamped->transform.translation.x = pose.position.x;
	transformStamped->transform.translation.y = pose.position.y;
	transformStamped->transform.translation.z = pose.position.z;
	transformStamped->transform.rotation.x = pose.orientation.x;
	transformStamped->transform.rotation.y = pose.orientation.y;
	transformStamped->transform.rotation.z = pose.orientation.z;
	transformStamped->transform.rotation.w = pose.orientation.w;
}

void AriacSensorManager::setPose
(const geometry_msgs::Pose & src, geometry_msgs::Pose & dstn) {
	dstn.position.x = src.position.x;
	dstn.position.y = src.position.y;
	dstn.position.z = src.position.z;
	dstn.orientation.x = src.orientation.x;
	dstn.orientation.y = src.orientation.y;
	dstn.orientation.z = src.orientation.z;
	dstn.orientation.w = src.orientation.w;
}

void AriacSensorManager::setPose
(geometry_msgs::TransformStamped * transformStamped,
		geometry_msgs::Pose &pose) {
	pose.position.x = transformStamped->transform.translation.x;
	pose.position.y = transformStamped->transform.translation.y;
	pose.position.z = transformStamped->transform.translation.z;
	pose.orientation.x = transformStamped->transform.rotation.x;
	pose.orientation.y = transformStamped->transform.rotation.y;
	pose.orientation.z = transformStamped->transform.rotation.z;
	pose.orientation.w = transformStamped->transform.rotation.w;
}

std::map<std::string,std::map<std::string,
std::vector<geometry_msgs::Pose>>> AriacSensorManager::getBinParts() {
	return all_binParts;
}


void AriacSensorManager::setTrackingPartInWorld() {
	setPose(tracking_pose_in_sensor, &transformStamped2_belt);
	br_s_c_belt.sendTransform(transformStamped2_belt);
	ros::Duration(0.01).sleep();
	try{
		transformStamped3_belt = tf_buffer_belt.lookupTransform("world", "logical_sensor_child",
				ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("exception");
		ROS_WARN("%s",ex.what());
		ros::Duration(0.01).sleep();
	}
	setPose(&transformStamped3_belt, tracking_part_->pose);
}

void AriacSensorManager::beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	if(order_manager_.isSegregated()){
		if(!order_manager_.isConveyorPartsPicked()) {
			auto conveyor_order = order_manager_.getConveyorOrderParts();
			//	ROS_INFO_STREAM(">> belt logical camera called." << conveyor_order.size());
			auto sensor_pose = image_msg->pose;
			auto current_time = ros::Time::now();

			transformStamped1_belt.header.stamp = current_time;
			transformStamped1_belt.header.frame_id = "world";
			transformStamped1_belt.child_frame_id = "logical_sensor";

			transformStamped2_belt.header.stamp = current_time;
			transformStamped2_belt.header.frame_id = "logical_sensor";
			transformStamped2_belt.child_frame_id = "logical_sensor_child";

			setPose(sensor_pose, &transformStamped1_belt);
			br_w_s_belt.sendTransform(transformStamped1_belt);
			ros::Duration(0.01).sleep();
			for (auto it = image_msg->models.begin(); it != image_msg->models.end();++it) {
				if (conveyor_order.count(it->type)) {
					//			ROS_INFO_STREAM(">> belt logical camera saw. " << it->type );
					auto part_type = it->type;

					if (tracking_part_ == nullptr) {
						tracking_part_ = new osrf_gear::Model();
						tracking_part_->type = it->type;
						tracking_part_->pose = it->pose;
						tracking_pose_in_sensor = it->pose;
						setTrackingPartInWorld();
					}

					else {
						if (it->type.compare(tracking_part_->type) == 0 && it-> pose.position.z > tracking_pose_in_sensor.position.z ) {  //error comapring world with sensor frame
							tracking_pose_in_sensor = it->pose;
							setTrackingPartInWorld();
						}
					}
				}
			}

			if (tracking_part_ != nullptr) {
				ROS_INFO_STREAM(">> Picking Part from Conveyor Belt" );
				order_manager_.pickPart(tracking_part_->pose, 0.12);
				ROS_INFO_STREAM(">> GOing TO Quality Camera !!" );
				order_manager_.getArmObject()->GoToQualityCamera();
				if(order_manager_.getArmObject()->isAtQualitySensor()) {
					if(is_faulty) {
						ROS_WARN_STREAM("Part is faulty");
						order_manager_.getArmObject()->dropInTrash();
						tracking_part_ = nullptr;
					} else {
						ROS_INFO_STREAM("Part is not faulty");
						AriacOrderPart* orderbeltPart;
						//				if(conveyor_order.count(tracking_part_->type)){
						orderbeltPart = conveyor_order[tracking_part_->type].back();

						//				}
						ROS_INFO_STREAM("Picking Order from belt of type : " << orderbeltPart->getPartType() <<
								" and delivering at " << orderbeltPart->getEndPose().position.x <<
								", " << orderbeltPart->getEndPose().position.y << ", " << orderbeltPart->getEndPose().position.z);

						dropInAGV(orderbeltPart->getEndPose());

						order_manager_.removeConveyorPart(orderbeltPart);
						tracking_part_ = nullptr;
					}
				}
			}
			if(order_manager_.getConveyorOrderParts().size() == 0) {
				//		ROS_INFO_STREAM("there are no conveyor parts" );
				order_manager_.setConveyorPartsPicked(true);

			}
		}
	}
}


void AriacSensorManager::updateFaultyPartPose(AriacOrderPart* part){
	if(faulty_part_ != nullptr){
		for (auto cam_it : all_binParts) {
			if(cam_it.second.count(part->getPartType())) {
				part->setCurrentPose(cam_it.second[part->getPartType()].back());
				return;
			}
		}
	}
	faulty_part_=nullptr;
}

void AriacSensorManager::binlogicalCameraCallback
(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg,
		const std::string& cam_name) {
	if(!task_completed){
		if(!order_manager_.isSegregated() || bin_part_faulty) {
			setAllBinParts(image_msg, cam_name);
			order_manager_.setBinCameraCalled();
			updateFaultyPartPose(faulty_part_);
			bin_part_faulty = false;
		}

		if (order_manager_.isConveyorPartsPicked() && !in_process) {
			in_process = true;
			auto bin_order = order_manager_.getBinOrderParts();
			//		auto conveyor_order = order_manager_.getConveyorOrderParts();
			//		ROS_INFO_STREAM(">> bin logical camera called." << conveyor_order.size());
			for(auto vec_it = bin_order.begin(); vec_it != bin_order.end(); ++vec_it){
				for(auto it = vec_it->second.begin(); it != vec_it->second.end(); ++it){
					auto current_pose = (*it)->getCurrentPose();
					auto end_pose = (*it)->getEndPose();
					ROS_INFO_STREAM(" Picking " << (*it)->getPartType() << " from bin.");
					ROS_WARN_STREAM(" Picking Position " << (*it)->getCurrentPose().position.x << "  " << (*it)->getCurrentPose().position.y << " from bin.");
					ROS_INFO_STREAM("Picking Order from belt of type : " << (*it)->getPartType() <<
													" and delivering at " << (*it)->getEndPose().position.x <<
													", " << (*it)->getEndPose().position.y << ", " << (*it)->getEndPose().position.z);
					order_manager_.pickfromBin(current_pose);
					ROS_INFO_STREAM(" Going to quality camera from bin");
					order_manager_.getArmObject()->GoToQualityCameraFromBin();
					if(order_manager_.getArmObject()->isAtQualitySensor()) {
						if(is_faulty) {
							ROS_WARN_STREAM("Part is faulty");
							bin_part_faulty =true;
							order_manager_.getArmObject()->dropInTrash();
							order_manager_.updateBinOrder(vec_it, it);
							AriacOrderPart* part = (*it);
							updateFaultyPartPose(part);
							return;
						} else {
							ROS_INFO_STREAM("Part is not faulty");
							ROS_INFO_STREAM("Dropping in AGV");
							dropInAGV(end_pose);
						}
					}
				}
			}
			in_process = false;
			task_completed = true;
		}

	} else {
		ros::Duration(.0).sleep();
		ROS_INFO_STREAM("Task Completed !!!");
		EndCompetition();
		ros::waitForShutdown();

	}

}

void AriacSensorManager::EndCompetition() {
	ros::ServiceClient start_client =
			sensor_nh_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

	if (!start_client.exists()) {
		ROS_INFO("Waiting for the competition to be End...");
		start_client.waitForExistence();
		ROS_INFO("Competition  now Ended.");
	}
	ROS_INFO("Requesting competition End...");
	std_srvs::Trigger srv;
	start_client.call(srv);
	if (!srv.response.success) {
		ROS_ERROR_STREAM(
				"Failed to End the competition: " << srv.response.message);
	} else {
		ROS_INFO("Competition Ended!");
	}
}


void AriacSensorManager::binlogicalCameraCallback1
(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg ) {
	binlogicalCameraCallback(image_msg, "cam1");
}

void AriacSensorManager::binlogicalCameraCallback2
(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	binlogicalCameraCallback(image_msg, "cam2");
}

void AriacSensorManager::agvLogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

}

void AriacSensorManager::SortAllBinParts() {
	sorted_all_binParts.clear();
	for(auto cam_id : all_binParts) {

		for(auto map_parts : cam_id.second) {
			auto part_type = map_parts.first;
			auto vec_parts = map_parts.second;
			if(sorted_all_binParts.count(part_type)) {
				sorted_all_binParts[part_type].insert(sorted_all_binParts[part_type].end(), vec_parts.begin(), vec_parts.end() );
			} else {
				sorted_all_binParts[part_type] = vec_parts;
			}
		}
	}
}

void AriacSensorManager::setAllBinParts(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg, std::string cam_name) {
	//	ROS_INFO_STREAM("setAllBinParts called");

	auto sensor_pose = image_msg->pose;
	auto current_time = ros::Time::now();
	geometry_msgs::TransformStamped* transformStamped1;
	geometry_msgs::TransformStamped* transformStamped2;
	geometry_msgs::TransformStamped* transformStamped3;
	tf2_ros::Buffer* tfBuffer;
	tf2_ros::TransformBroadcaster* br_w_s;
	tf2_ros::TransformBroadcaster* br_s_c;

	if(cam_name.compare("cam1") == 0){
		transformStamped1 = &transformStamped1_bin1;
		transformStamped2 = &transformStamped2_bin1;
		transformStamped3 = &transformStamped3_bin1;
		br_w_s = &br_w_s_bin1;
		br_s_c = &br_s_c_bin1;
		tfBuffer = &tf_buffer_bin1;
	}
	else if(cam_name.compare("cam2") == 0){
		transformStamped1 = &transformStamped1_bin2;
		transformStamped2 = &transformStamped2_bin2;
		transformStamped3 = &transformStamped3_bin2;
		br_w_s = &br_w_s_bin2;
		br_s_c = &br_s_c_bin2;
		tfBuffer = &tf_buffer_bin2;
	}
	transformStamped1->header.stamp = current_time;
	transformStamped1->header.frame_id = "world";
	transformStamped1->child_frame_id = cam_name;

	transformStamped2->header.stamp = current_time;
	transformStamped2->header.frame_id = cam_name;
	auto child_string = cam_name + "logical_sensor_child";
	transformStamped2->child_frame_id = child_string;

	setPose(sensor_pose,transformStamped1);
	br_w_s->sendTransform(*transformStamped1);
	ros::Duration(0.001).sleep();
	if(all_binParts.count(cam_name) == 1) {
		all_binParts[cam_name].clear();
	}
	for(auto it =image_msg->models.begin(); it!=image_msg->models.end();++it) {
		setPose( it->pose, transformStamped2);
		br_s_c->sendTransform(*transformStamped2);
		ros::Duration(0.001).sleep();
		auto partType = it->type;
		try {
			*transformStamped3 = tfBuffer->lookupTransform("world", child_string,
					ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("exception");
			ROS_WARN("%s",ex.what());
			ros::Duration(0.001).sleep();
		}
		geometry_msgs::Pose pose;
		setPose(transformStamped3, pose);
		all_binParts[cam_name][partType].push_back(pose);

	}
	SortAllBinParts();
	order_manager_.setBinCameraCalled();

}



void AriacSensorManager::qualityControlSensorCallback
(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg) {
	is_faulty = !image_msg->models.empty();
}

void AriacSensorManager::dropInAGV(const geometry_msgs::Pose& end_pose){
	auto target_pose_ = end_pose;
	target_pose_.position.z += 0.05;
	order_manager_.getArmObject()->GoToTarget(target_pose_);
	ros::Duration(0.1).sleep();
	order_manager_.getArmObject()->GripperToggle(false);
	ros::Duration(0.1).sleep();
	target_pose_.position.z += 0.4;
	order_manager_.getArmObject()->GoToTarget(target_pose_);
}
