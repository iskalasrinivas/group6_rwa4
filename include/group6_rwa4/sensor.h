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


class AriacOrderManager;

class AriacSensorManager {


private:
    std::map<geometry_msgs::Pose, std::map<std::string, std::vector<geometry_msgs::Pose>>> all_binParts;
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
    std::map<geometry_msgs::Pose, std::map<std::string, std::vector<geometry_msgs::Pose>>> getBinParts();
    void computeWorldTransformation(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
	void beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);

	void breakBeamCallback(const osrf_gear::Proximity::ConstPtr &);
	bool isObjectDetected();

};

#endif //GROUP6_RWA4_SENSOR
