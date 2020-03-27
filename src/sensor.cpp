//
// Created by zeid on 2/27/20.
//
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
        if(!order_manager_.get_conveyor_order_part().empty()){
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


// if all order part of conveyor belt picked then
if (all order part of conveyor belt picked) {
    // convert image_msg to world frame

    //call pick part to pick that part

    // if attached take it to quality camera

    //if attached and if no faulty part  deliver it to agv end pose


    //if faulty deliver it to trash bin
}
// if all order part of bin belt picked then send agv and end competition

order_manager_.setBinCameraCalled();
}


// void AriacSensorManager::binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//     auto sensor_pose = image_msg->pose;
//     auto current_time = ros::Time::now();
//     tf2_ros::TransformListener tfListener(tfBuffer);


//     transformStamped1.header.stamp = current_time;
//     transformStamped1.header.frame_id = "world";
//     transformStamped1.child_frame_id = "logical_sensor";

//     transformStamped2.header.stamp = current_time;
//     transformStamped2.header.frame_id = "logical_sensor";
//     transformStamped2.child_frame_id = "logical_sensor_child";

//     setPose(sensor_pose,transformStamped1);
//     br_w_s.sendTransform(transformStamped1);
//     ros::Duration(0.001).sleep();
//     all_binParts[sensor_pose].clear();
//     for(auto it =image_msg->models.begin(); it!=image_msg->models.end();++it) {
//         setPose( it->pose, transformStamped2);
//         br_s_c.sendTransform(transformStamped2);
//         ros::Duration(0.001).sleep();
//         auto partType = it->type;
//         try{
//             transformStamped3 = tfBuffer.lookupTransform("world", "logical_sensor_child",
//                                                          ros::Time(0));
//             geometry_msgs::Pose pose;
//             setPose(transformStamped3, pose);
//             all_binParts[sensor_pose][partType].push_back(pose);

//         }
//         catch (tf2::TransformException &ex) {
//             ROS_WARN("exception");
//             ROS_WARN("%s",ex.what());
//             ros::Duration(0.001).sleep();
//         }


//     }
//     order_manager_.setBinCameraCalled();

// }


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





