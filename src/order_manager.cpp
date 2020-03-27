//
// Created by zeid on 2/27/20.
//


#include <osrf_gear/AGVControl.h>
#include <string>
#include <initializer_list>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <boost/optional.hpp>
#include "order_manager.h"


//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager (std::map<geometry_msgs::Pose, std::map<std::string, std::vector<geometry_msgs::Pose>>>* abp):  arm1_{"arm1"}, all_bin_parts(abp),
isBinCameraCalled(false){
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
	while(!isBinCameraCalled){
        ROS_INFO_STREAM("bin camera is not called" << std::endl);
    }
    segregateOrders();
}

void AriacOrderManager::setBinCameraCalled(){
    isBinCameraCalled = true;
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

bool AriacOrderManager::in_vicinity(const geometry_msgs::TransformStamped& msg){
	double threshold_z = 0.1;
	double threshold_y = 0.35;
	return (arm1_.getHomeCartPose().position.z- msg.transform.translation.z < threshold_z &&
						arm1_.getHomeCartPose().position.y- msg.transform.translation.y < threshold_y);
}

void AriacOrderManager::pick_part(const geometry_msgs::TransformStamped& msg, int y) {
	if(task_pending) {
		ROS_INFO("robot_controller_pathPlanning");
		double threshold_z = 0.1;
		double threshold_y = 0.35;
		geometry_msgs::Pose world_part_pose;
        world_part_pose.position.x= msg.transform.translation.x;
        world_part_pose.position.y= msg.transform.translation.y - y;
        world_part_pose.position.z= msg.transform.translation.z;
        world_part_pose.orientation.x= msg.transform.rotation.x;
        world_part_pose.orientation.y= msg.transform.rotation.y;
        world_part_pose.orientation.z= msg.transform.rotation.z;
        world_part_pose.orientation.w = msg.transform.rotation.w;


        pickPart(world_part_pose);
	}

}




void AriacOrderManager::pickPart(geometry_msgs::Pose world_part_pose){
    if(!arm1_.isPartAttached()) {
        //		ROS_INFO("part not attached");
        //		ROS_INFO_STREAM(msg.transform.translation.x<<","<< msg.transform.translation.y<<","<< msg.transform.translation.z);
        arm1_.GoToTarget(world_part_pose);
        //		ROS_INFO("going toward part");
        ROS_INFO_STREAM("gap: "<<arm1_.getHomeCartPose().position.z- msg.transform.translation.z << ","<< arm1_.getHomeCartPose().position.y- msg.transform.translation.y);
        if(in_vicinity()) {
            //			arm1_.GoToTarget(world_part_pose);
            // arm1_.PickPart(world_part_pose);
            arm1_.GripperToggle(true);
            world_part_pose.position.z += 0.2;
            world_part_pose.position.y += y;   // 0.5
            arm1_.GoToTarget(world_part_pose);
        }
    } else {
        arm1_.GoToEnd();
        task_pending = false;
    }
}


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

std::map<std::string, std::vector<AriacOrderPart>> AriacOrderManager::getBinOrderParts(){
    return bin_order_parts;
}

void AriacOrderManager::remove_conveyor_part(AriacOrderPart* orderPart) {
    auto part_type = orderPart->get_part_type();
	if(conveyor_order_parts.size() != 0){
		if(conveyor_order_parts[part_type].size() != 0) {
            if (conveyor_order_parts[part_type].size() == 0) {
                conveyor_order_parts.erase(part_type);
            }
        }
	}
}

void AriacOrderManager::remove_bin_part(AriacOrderPart* orderPart) {
    auto part_type = orderPart->get_part_type();
    if (bin_order_parts.size() != 0) {
        if (bin_order_parts[part_type].size() != 0) {
            bin_order_parts[part_type].pop_back();
            if (bin_order_parts[part_type].size() == 0) {
                bin_order_parts.erase(part_type);
            }
        }
    }
}

void AriacOrderManager::drop_part_to_agv(){
    if(part_is_faulty){
        move_to_target(faulty_bin_pose);
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
		waypoints.emplace_back(current_pose);
	}
	arm1_.GoToTarget(waypoints);
}

void AriacOrderManager::pathplanning(const geometry_msgs::TransformStamped& msg) {
	segregateorders(); // @TODO Srinivas segregate parts from order into two vectors  bin_order_parts and conveyor_order_parts

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
