//
// Created by zeid on 3/1/20.
//
#ifndef GROUP6_RWA4_ARIAC_ORDER_PART
#define GROUP6_RWA4_ARIAC_ORDER_PART
#include <string>
#include <geometry_msgs/Pose.h>

class AriacOrderPart {
private:
	std::string part_type_;
	geometry_msgs::Pose end_pose_;
	geometry_msgs::Pose current_pose_;

public:
	AriacOrderPart();
	~AriacOrderPart();


	void set_part_type(std::string);
	void set_end_pose(geometry_msgs::Pose);
	void set_current_pose(geometry_msgs::Pose);
	const std::string get_part_type();
	const geometry_msgs::Pose get_end_pose();
	const geometry_msgs::Pose get_current_pose();


};
#endif //GROUP6_RWA4_ARIAC_PART_MANAGER
