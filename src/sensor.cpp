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

#include "sensor.h"


AriacSensorManager::AriacSensorManager() : order_manager_(& all_binParts) {
	ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10 ,
                                                           &AriacSensorManager::binlogicalCameraCallback, this);
	camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10 ,
	                                                  &AriacSensorManager::beltlogicalCameraCallback, this);
    camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10 ,
                                                &AriacSensorManager::binlogicalCameraCallback, this);
    quality_control_camera_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10 , &AriacSensorManager::qualityControlSensor1Callback, this);
}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::qualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg){

}

void AriacSensorManager::setPose( const geometry_msgs::Pose src, geometry_msgs::Pose & dstn){
	dstn.position.x = src.position.x;
	dstn.position.y = src.position.y;
	dstn.position.z = src.position.z;
	dstn.orientation.x = src.orientation.x;
	dstn.orientation.y = src.orientation.y;
	dstn.orientation.z = src.orientation.z;
	dstn.orientation.w = src.orientation.w;
}

void AriacSensorManager::setPose(const geometry_msgs::Pose pose, geometry_msgs::TransformStamped &transformStamped ){
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	transformStamped.transform.rotation.x = pose.orientation.x;
	transformStamped.transform.rotation.y = pose.orientation.y;
	transformStamped.transform.rotation.z = pose.orientation.z;
	transformStamped.transform.rotation.w = pose.orientation.w;
}

void AriacSensorManager::setPose(const geometry_msgs::TransformStamped transformStamped, geometry_msgs::Pose &pose){
     pose.position.x = transformStamped.transform.translation.x;
     pose.position.y = transformStamped.transform.translation.y;
     pose.position.z = transformStamped.transform.translation.z;
     pose.orientation.x = transformStamped.transform.rotation.x;
     pose.orientation.y = transformStamped.transform.rotation.y;
     pose.orientation.z = transformStamped.transform.rotation.z;
     pose.orientation.w = transformStamped.transform.rotation.w;
}

std::map<geometry_msgs::Pose, std::map<std::string, std::vector<geometry_msgs::Pose>>> AriacSensorManager::getBinParts(){
     return all_binParts;
}

void AriacSensorManager::computeWorldTransformation(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    auto sensor_pose = image_msg->pose;
    auto current_time = ros::Time::now();
    tf2_ros::TransformListener tfListener(tfBuffer);


    transformStamped1.header.stamp = current_time;
    transformStamped1.header.frame_id = "world";
    transformStamped1.child_frame_id = "logical_sensor";

    transformStamped2.header.stamp = current_time;
    transformStamped2.header.frame_id = "logical_sensor";
    transformStamped2.child_frame_id = "logical_sensor_child";

    setPose(sensor_pose, transformStamped1);
    br_w_s.sendTransform(transformStamped1);
    ros::Duration(0.001).sleep();
    for (auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it) {
        setPose(it->pose, transformStamped2);
        br_s_c.sendTransform(transformStamped2);
        ros::Duration(0.001).sleep();
        try {
            transformStamped3 = tfBuffer.lookupTransform("world", "logical_sensor_child",
                                                         ros::Time(0));

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("exception");
            ROS_WARN("%s", ex.what());
            ros::Duration(0.001).sleep();
        }


    }
}




void AriacSensorManager::beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
	// order type same as camera type
	    // computeWorldTransformation(image_msg);
		//order_manager_.pathplanning(transformStamped3);
        if(!order_manager_.getConveyorOrderParts().empty()){
            // convert image_msg to world frame

            //call pick part to pick that part

            // if attached take it to quality camera

            //if attached and if no faulty part  deliver it to agv end pose


            //if faulty deliver it to trash bin
        } else {
            //set bool :all order part of conveyor belt picked
        }


	}

void AriacSensorManager::binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
// all bin part is empty fill the all bin part for segregate purpose
    setAllBinParts(image_msg);
// if all order part of conveyor belt picked then
if (conveyor_parts_picked) {

	auto bin_order = order_manager_.getBinOrderParts();
	for(auto map_it = bin_order.begin(); map_it != bin_order.end(); map_it++){
		for(auto it = map_it->second.begin(); it != map_it->second.end(); it++){
			auto current_pose = it->getCurrentPose();
            order_manager_.pickPart(current_pose, 0);
		}
	}
    //call pick part to pick that part
    

    // if attached take it to quality camera

    //if attached and if no faulty part  deliver it to agv end pose


    //if faulty deliver it to trash bin
}
// if all order part of bin belt picked then send agv and end competition


}


void AriacSensorManager::setAllBinParts(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    auto sensor_pose = image_msg->pose;
    auto current_time = ros::Time::now();
    tf2_ros::TransformListener tfListener(tfBuffer);


    transformStamped1.header.stamp = current_time;
    transformStamped1.header.frame_id = "world";
    transformStamped1.child_frame_id = "logical_sensor";

    transformStamped2.header.stamp = current_time;
    transformStamped2.header.frame_id = "logical_sensor";
    transformStamped2.child_frame_id = "logical_sensor_child";

    setPose(sensor_pose,transformStamped1);
    br_w_s.sendTransform(transformStamped1);
    ros::Duration(0.001).sleep();
    all_binParts[sensor_pose].clear();
    for(auto it =image_msg->models.begin(); it!=image_msg->models.end();++it) {
        setPose( it->pose, transformStamped2);
        br_s_c.sendTransform(transformStamped2);
        ros::Duration(0.001).sleep();
        auto partType = it->type;
        try{
            transformStamped3 = tfBuffer.lookupTransform("world", "logical_sensor_child",
                                                         ros::Time(0));
            geometry_msgs::Pose pose;
            setPose(transformStamped3, pose);
            all_binParts[sensor_pose][partType].push_back(pose);

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("exception");
            ROS_WARN("%s",ex.what());
            ros::Duration(0.001).sleep();
        }


    }
    order_manager_.setBinCameraCalled();

}


bool AriacSensorManager::isObjectDetected() {
	return object_detected;
}



void AriacSensorManager::breakBeamCallback(const osrf_gear::Proximity::ConstPtr & msg) {

	if (msg->object_detected) {  // If there is an object in proximity.
		ROS_INFO("Break beam triggered.");
		object_detected = true;
	}
	else{
		object_detected = false;
	}
}





