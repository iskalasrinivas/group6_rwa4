
#ifndef GROUP6_RWA4_ORDER_MANAGER
#define GROUP6_RWA4_ORDER_MANAGER

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>

#include "robot_controller.h"
#include "ariac_order_part.h"
//#include "sensor.h"

// class AriacSensorManager;
//class RobotController;
using std::vector;

class AriacOrderManager {

private:
	ros::NodeHandle order_manager_nh_;
	ros::Subscriber order_subscriber_;
	std::vector<osrf_gear::Order> received_orders_;
	vector<AriacOrderPart> part_manager;
	std::vector<std::string> product_type;
//	ros::Subscriber wayPoint_subscriber;
	RobotController arm1_;
	//    RobotController arm2_;
	tf::TransformListener part_tf_listener_;
	std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
	std::string object;
	std::map<std::string, std::vector<std::string>> product_frame_list_;
	osrf_gear::Order order_;
	bool task_pending;

//	RobotController sensors_;

public:
	AriacOrderManager();
	~AriacOrderManager();
	void OrderCallback(const osrf_gear::Order::ConstPtr&);
	void ExecuteOrder();
	boost::optional<std::string> GetProductFrame(std::string);
	std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
	bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose>,int );
	std::vector<std::string> getProductType();
	void setProductType();
	void SubmitAGV(int);
	ros::NodeHandle* getnode();
	void remove_conveyor_part();
	void remove_bin_part();
	void drop_part_to_agv();
	void move_to_target(geometry_msgs::Pose final_pose);

	void pathplanning(const geometry_msgs::TransformStamped&);
};
#endif //GROUP6_RWA4_ORDER_MANAGER

