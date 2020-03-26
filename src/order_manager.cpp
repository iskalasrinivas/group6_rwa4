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
AriacOrderManager::AriacOrderManager(std::map<geometry_msgs::Pose, std::map<std::string, std::vector<geometry_msgs::Pose>>>* abp):  arm1_{"arm1"}, all_bin_parts(abp){
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
	setOrderParts();
}

void AriacOrderManager::setOrderParts(){
	ROS_INFO_STREAM("reading order." << std::endl);

	for (const auto &order:received_orders_) {
		auto order_id = order.order_id;
		auto shipments = order.shipments;
		for (const auto &shipment: shipments) {
			auto shipment_type = shipment.shipment_type;
			auto products = shipment.products;

			for (const auto &product: products) {
                std::string part_type = product.type;
                if(all_orderParts.count(part_type)){
                    auto vector = all_orderParts[part_type];
                    AriacOrderPart order_part;
                    order_part.set_part_type (product.type);
                    order_part.set_end_pose (product.pose);
                    vector.push_back(order_part);
                } else{
                    AriacOrderPart order_part;
                    order_part.set_part_type (product.type);
                    order_part.set_end_pose (product.pose);
                    std::vector<AriacOrderPart> vec;
                    vec.push_back(order_part);
                    all_orderParts.insert({part_type, vec});

                }

			}
		}
	}
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

std::map<geometry_msgs::Pose, std::map<std::string, std::vector<geometry_msgs::Pose>>>* all_bin_parts;
std::map<std::string, std::vector<AriacOrderPart>> all_orderParts;
std::map<std::string, std::vector<AriacOrderPart>> conveyor_order_parts;
std::map<std::string, std::vector<AriacOrderPart>> bin_order_parts;

void AriacOrderManager::setCurrentPose(std::vector<AriacOrderPart> &ariacOrderparts,
                                                const std::vector<geometry_msgs::Pose> &vecPose) {
    std::vector<geometry_msgs::Pose>::const_iterator  it_vecPose = vecPose.begin();
    for (auto &orderPart: ariacOrderparts) {
        orderPart.set_current_pose(*it_vecPose);
        ++it_vecPose;

    }
}

void AriacOrderManager::segregateOrders(){

    for (const auto &orderPart: all_orderParts){
        for(const auto &binPart: *all_bin_parts){
            auto oVecPart = orderPart.second;
            auto oType = orderPart.first;
            auto binMapPart = binPart.second;
            for(auto bPart: binMapPart){
                if(oType == bPart.first && bPart.second.size() > oVecPart.size()){
                    setCurrentPose(oVecPart, bPart.second);
                    bin_order_parts.insert({oType, oVecPart});
                }
                else{
                    conveyor_order_parts.insert({oType, oVecPart});
                }

            }

        }
    }

}

void AriacOrderManager::pathplanning(const geometry_msgs::TransformStamped& msg) {
	segregateorders(); // @TODO Srinivas segregate parts from order into two vectors  bin_order_parts and conveyor_order_parts

	while(conveyor_order_parts.size() != 0) {


		pick_part_from_conveyor(msg);   //@TODO pick part form the conveyer belt Line 209-245 @ Preyash
		std::string picked_part_id = identify_part(); // @TODO @ Sanket
		remove_part(picked_part_id);  // @TODO Dinesh
		drop_part_to_agv(); // @TODO Dinesh
		move_to_home_position(); // @TODO Saurav

	}

	while(bin_order_parts.size() != 0) {


			pick_part_from_bin(msg);   //@TODO pick part form the conveyer belt Line 209-245 @ Preyash
			std::string picked_part_id = identify_part(); // @TODO @ Sanket
			remove_part(picked_part_id);  // @TODO Dinesh
			drop_part_to_agv(); // @TODO Dinesh
			move_to_home_position();// @TODO Saurav

		}

}
