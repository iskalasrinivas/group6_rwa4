//
// Created by zeid on 2/27/20.
//


#include <osrf_gear/AGVControl.h>
#include <string>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/optional.hpp>
#include "order_manager.h"


//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager():  arm1_{"arm1"} {
	//	order_manager_nh_= nh;
	//	wayPoint_subscriber = order_manager_nh_.subscribe(
	//			"/ariac/logical_sensor_4/tracking_object", 10, &AriacOrderManager::pathplanningCallback, this);
	order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10,
			&AriacOrderManager::OrderCallback, this);
	task_pending = true;

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
	ROS_WARN(">>>>> OrderCallback");
	received_orders_.push_back(*order_msg);
	ROS_INFO_STREAM("no of orders "<< received_orders_.size() << std::endl);
	setProductType();
}

void AriacOrderManager::setProductType(){
	ROS_INFO_STREAM("reading order." << std::endl);

	for (const auto &order:received_orders_) {
		auto order_id = order.order_id;
		auto shipments = order.shipments;
		for (const auto &shipment: shipments) {
			auto shipment_type = shipment.shipment_type;
			auto products = shipment.products;

			for (const auto &product: products) {
				product_type_pose_.first = product.type;
				product_type.push_back(product.type);

			}
		}
	}
	ROS_INFO_STREAM("no of products "<< product_type.size() << std::endl);


}

std::vector<std::string> AriacOrderManager::getProductType(){


	return product_type;
}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
boost::optional<std::string> AriacOrderManager::GetProductFrame(std::string product_type) {
	//--Grab the last one from the list then remove it

	if (!product_frame_list_.empty()) {
		std::string frame = product_frame_list_[product_type].back();
		ROS_INFO_STREAM("Frame >>>> " << frame);
		product_frame_list_[product_type].pop_back();
		return frame;
	} else {
		ROS_ERROR_STREAM("No product frame found for " << product_type);
		ros::shutdown();
		return boost::optional<std::string>{};
	}

}



//bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
//    std::string product_type = product_type_pose.first;
//    ROS_WARN_STREAM("Product type >>>> " << product_type);
//    std::string product_frame = this->GetProductFrame(product_type);
//    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
//    auto part_pose = camera_.GetPartPose("/world",product_frame);
//
//
//    if(product_type == "pulley_part")
//        part_pose.position.z += 0.08;
//    //--task the robot to pick up this part
//    bool failed_pick = arm1_.PickPart(part_pose);
//    ROS_WARN_STREAM("Picking up state " << failed_pick);
//    ros::Duration(0.5).sleep();
//
//    while(!failed_pick){
//        auto part_pose = camera_.GetPartPose("/world",product_frame);
//        failed_pick = arm1_.PickPart(part_pose);
//    }
//
//    //--get the pose of the object in the tray from the order
//    geometry_msgs::Pose drop_pose = product_type_pose.second;
//
//    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
//
//    if(agv_id==1){
//        StampedPose_in.header.frame_id = "/kit_tray_1";
//        StampedPose_in.pose = drop_pose;
//        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
//        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
//        StampedPose_out.pose.position.z += 0.1;
//        StampedPose_out.pose.position.y -= 0.2;
//        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
//
//    }
//    else{
//        StampedPose_in.header.frame_id = "/kit_tray_2";
//        StampedPose_in.pose = drop_pose;
//        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
//        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
//        StampedPose_out.pose.position.z += 0.1;
//        StampedPose_out.pose.position.y += 0.2;
//        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
//    }
//    auto result = arm1_.DropPart(StampedPose_out.pose);
//
//    return result;
//}


//void AriacOrderManager::ExecuteOrder() {
//    ROS_WARN(">>>>>> Executing order...");
//    //scanned_objects_ = camera_.GetParts();
//
//    //-- used to check if pick and place was successful
//    bool pick_n_place_success{false};
//
//    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
//
//    ros::spinOnce();
//    ros::Duration(1.0).sleep();
//    product_frame_list_ = camera_.get_product_frame_list();
//    for (const auto &order:received_orders_){
//        auto order_id = order.order_id;
//        auto shipments = order.shipments;
//        for (const auto &shipment: shipments){
//            auto shipment_type = shipment.shipment_type;
//            auto agv = shipment.agv_id.back();//--this returns a char
//            //-- if agv is any then we use AGV1, else we convert agv id to int
//            //--agv-'0' converts '1' to 1 and '2' to 2
//            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
//
//            auto products = shipment.products;
//            ROS_INFO_STREAM("Order ID: " << order_id);
//            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
//            ROS_INFO_STREAM("AGV ID: " << agv_id);
//            for (const auto &product: products){
//                ros::spinOnce();
//                product_type_pose_.first = product.type;
//                product_type.push_back(product.type);
//                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
//                product_type_pose_.second = product.pose;
//                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
//                pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);
//                //--todo: What do we do if pick and place fails?
//            }
//            SubmitAGV(1);
//            ROS_INFO_STREAM("Submitting AGV 1");
//            int finish=1;
//        }
//
//
//    }
//}



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
	} else
		ROS_INFO("Service succeeded.");
}


ros::NodeHandle* AriacOrderManager::getnode() {
	return &order_manager_nh_;
}
//

//void AriacOrderManager::pick_part(const geometry_msgs::TransformStamped& msg) {
//	if(task_pending) {
//		ROS_INFO("robot_controller_pathPlanning");
//		double threshold_z = 0.1;
//		double threshold_y = 0.35;
//		geometry_msgs::Pose arm_base_part_pose;
//		arm_base_part_pose.position.x= msg.transform.translation.x;
//		arm_base_part_pose.position.y= msg.transform.translation.y-0.2;
//		arm_base_part_pose.position.z= msg.transform.translation.z;
//		arm_base_part_pose.orientation.x= msg.transform.rotation.x;
//		arm_base_part_pose.orientation.y= msg.transform.rotation.y;
//		arm_base_part_pose.orientation.z= msg.transform.rotation.z;
//		arm_base_part_pose.orientation.w = msg.transform.rotation.w;
//		//	if(count ==0) {
//		//	ROS_INFO_STREAM("isPartAttached status" << arm1_.isPartAttached());
//		if(!arm1_.isPartAttached()) {
//			//		ROS_INFO("part not attached");
//			//		ROS_INFO_STREAM(msg.transform.translation.x<<","<< msg.transform.translation.y<<","<< msg.transform.translation.z);
//			arm1_.GoToTarget(arm_base_part_pose);
//			//		ROS_INFO("going toward part");
//			ROS_INFO_STREAM("gap: "<<arm1_.getHomeCartPose().position.z- msg.transform.translation.z << ","<< arm1_.getHomeCartPose().position.y- msg.transform.translation.y);
//			if(arm1_.getHomeCartPose().position.z- msg.transform.translation.z < threshold_z &&
//					arm1_.getHomeCartPose().position.y- msg.transform.translation.y < threshold_y) {
//				//			arm1_.GoToTarget(arm_base_part_pose);
//				// arm1_.PickPart(arm_base_part_pose);
//				arm1_.GripperToggle(true);
//				arm_base_part_pose.position.z += 0.2;
//				arm_base_part_pose.position.y += 0.5;
//				arm1_.GoToTarget(arm_base_part_pose);
//			}
//		} else {
//			arm1_.GoToEnd();
//			task_pending = false;
//		}
//	}
//
//}

void AriacOrderManager::remove_conveyor_part(std::string picked_part_id) {

	if(conveyor_order_parts[picked_part_id].count != 0){
		if(conveyor_order_parts[picked_part_id].size() != 0){
			conveyor_order_parts[picked_part_id].pop_back();
			}
		else{
			conveyor_order_parts.erase(picked_part_id);
		}
	}
}

void AriacOrderManager::remove_bin_part(std::string picked_part_id) {

	if(bin_order_parts[picked_part_id].count != 0){
		if(bin_order_parts[picked_part_id].size() != 0){
			bin_order_parts[picked_part_id].pop_back();
			}
		else{
			bin_order_parts.erase(picked_part_id);
		}
	}
}


void AriacOrderManager::drop_part_to_agv(){
	geometry_msgs::Pose quality_control_camera_pose;
	geometry_msgs::Pose agv_pose;
	// & ariac order part (end pose) 

	geometry_msgs::Pose faulty_pose;
    move_to_target(quality_control_camera_pose);

    bool faulty_parts = false;
    if(faulty_parts = true){
    	move_to_target(faulty_bin);
    	GripperToggle = false;
    }
    else{
    	move_to_target(agv_pose);
    	GripperToggle = true;
    }
}

void AriacOrderManager::move_to_target(geometry_msgs::Pose final_pose){
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose dt_pose;

	dt_pose.position.x = (final_pose.position.x - arm1_.getHomeCartPose().position.x) / 10;
	dt_pose.position.y = (final_pose.position.y - arm1_.getHomeCartPose().position.y) / 10;
	dt_pose.position.z = (final_pose.position.z - arm1_.getHomeCartPose().position.z) / 10;
	dt_pose.orientation.x = 0;
	dt_pose.orientation.y = 0;
	dt_pose.orientation.z = 0;
	dt_pose.orientation.w = 0;

	for(int i = 1; i <= 10; i++){
		geometry_msgs::Pose current_pose = arm1_.getHomeCartPose() + i * dt_pose;
		waypoints.push_back(current_pose);	
	}
	GoToTarget(waypoints);
}


void AriacOrderManager::pathplanning(const geometry_msgs::TransformStamped& msg) {
	segrgateorders(); // @TODO Srinivas segregate parts from order into two vectors  bin_order_parts and conveyor_order_parts

	while(conveyor_order_parts.size() != 0) {


		pick_part_from_conveyor(msg);   //@TODO pick part form the conveyer belt Line 209-245 @ Preyash
		std::string picked_part_id = identify_part(); // @TODO @ Sanket
		remove_conveyor_part(picked_part_id);  // @TODO Dinesh
		drop_part_to_agv(); // @TODO Dinesh
		move_to_home_position(); // @TODO Saurav

	}

	while(bin_order_parts.size() != 0) {


			pick_part_from_bin(msg);   //@TODO pick part form the conveyer belt Line 209-245 @ Preyash
			std::string picked_part_id = identify_part(); // @TODO @ Sanket
			remove_bin_part(picked_part_id);  // @TODO Dinesh
			drop_part_to_agv(); // @TODO Dinesh
			move_to_home_position();// @TODO Saurav

		}

}
