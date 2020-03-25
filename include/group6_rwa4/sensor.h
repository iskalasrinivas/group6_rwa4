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

using std::vector;

//class AriacOrderManager;

class AriacSensorManager {


private:
	AriacOrderManager* order_manager_;


	ros::NodeHandle sensor_nh_;

	ros::Subscriber camera_1_subscriber_;
	ros::Subscriber camera_4_subscriber_;
	ros::Subscriber camera_5_subscriber_;
	ros::Subscriber breakbeam_subscriber;

	geometry_msgs::TransformStamped transformStamped1;
	geometry_msgs::TransformStamped transformStamped2;
	geometry_msgs::TransformStamped transformStamped3;
	tf2_ros::Buffer tfBuffer;
    std::map<std::string, std::map<std::string, > all_binParts;


	tf2_ros::TransformBroadcaster br_w_s;
	tf2_ros::TransformBroadcaster br_s_c;
	bool object_detected = false;


	//	tf::TransformListener camera_tf_listener_;
	//	tf::StampedTransform camera_tf_transform_;
	//	std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
	//	std::map<std::string, std::vector<std::string>> product_frame_list_;
public:
	AriacSensorManager(AriacOrderManager* );
	~AriacSensorManager();
	void setPose(const geometry_msgs::Pose pose, geometry_msgs::TransformStamped &);
	void setPose(const geometry_msgs::Pose pose, geometry_msgs::Pose &);
	void beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void breakBeamCallback(const osrf_gear::Proximity::ConstPtr &);
	bool isObjectDetected();

};

#endif //GROUP6_RWA4_SENSOR
