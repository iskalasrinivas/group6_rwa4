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
	geometry_msgs::Pose part_pose_;
	int part_frame_;

public:
	AriacOrderPart();
	~AriacOrderPart();


	void set_part_type(std::string);
	void set_part_frame(int);
	void set_part_pose(geometry_msgs::Pose);
	const std::string get_part_type();
	const int get_part_frame();
	const geometry_msgs::Pose get_part_pose();


};
#endif //GROUP6_RWA4_ARIAC_PART_MANAGER
