/**
 * @file      src/robot_controller.cpp
 * @brief     Source file for Robot Controller
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

#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <robot_controller.h>

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) : robot_controller_nh_("/ariac/"+arm_id), robot_controller_options("manipulator",
		"/ariac/"+arm_id+"/robot_description", robot_controller_nh_), robot_move_group_(robot_controller_options) {

	ROS_WARN(">>>>> RobotController");

	// setting parameters of planner
	robot_move_group_.setPlanningTime(20);
	robot_move_group_.setNumPlanningAttempts(10);
	robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
	robot_move_group_.setMaxVelocityScalingFactor(0.9);
	robot_move_group_.setMaxAccelerationScalingFactor(0.9);
	// robot_move_group_.setEndEffector("moveit_ee");
	robot_move_group_.allowReplanning(true);
	collisionAvoidance();
	home_joint_pose_ =  {0.1, 3.14,  -2.7,-1.0, 2.1, -1.59, 0.126};
	quality_cam_joint_position_ = {1.18, 1.26,  -0.38, 1.13, 2.26, -1.51, 0.0};
	trash_bin_joint_position_ = {1.18, 3.02,  -0.63, -2.01, 3.52, -1.51, 0.0};
	offset_ = 0.025;

	//--topic used to get the status of the gripper
	gripper_subscriber_ = gripper_nh_.subscribe(
			"/ariac/arm1/gripper/state", 10, &RobotController::GripperCallback, this);


	SendRobotHome();

	robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
			ros::Time(0), ros::Duration(10));
	robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
			ros::Time(0), robot_tf_transform_);


	fixed_orientation_.x = robot_tf_transform_.getRotation().x();
	fixed_orientation_.y = robot_tf_transform_.getRotation().y();
	fixed_orientation_.z = robot_tf_transform_.getRotation().z();
	fixed_orientation_.w = robot_tf_transform_.getRotation().w();

	tf::quaternionMsgToTF(fixed_orientation_,q);
	tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


	//	end__joint_position_ = {1.5, 1.5, -0.9, 1.9, 3.1, -1.59, 0.126};

	//	end_position_[0] = 2.2;
	//    end_position_[1] = 4.5;
	//    end_position_[2] = 1.2;
	//    end_pose_.position.x = 0.0;
	//    end_pose_.position.y = 0.0;
	//    end_pose_.position.z = 0.0;
	//    end_pose_.orientation = fixed_orientation_;



	robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
			ros::Duration(10));
	robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
			robot_tf_transform_);

	home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
	home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
	home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
	home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
	home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
	home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
	home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

	agv_tf_listener_.waitForTransform("world", "kit_tray_1",
			ros::Time(0), ros::Duration(10));
	agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
			ros::Time(0), agv_tf_transform_);
	agv_position_.position.x = agv_tf_transform_.getOrigin().x();
	agv_position_.position.y = agv_tf_transform_.getOrigin().y();
	agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

	gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
			"/ariac/arm1/gripper/control");
	counter_ = 0;
	drop_flag_ = false;

	static_bin_pose.position.x = -0.20;
	static_bin_pose.position.y = 0.0;
	static_bin_pose.position.z = 0.95;
	static_bin_pose.orientation = fixed_orientation_;
	// r: 2.392647, p:1.438845, y:0.522735;
	quality_static_pose.position.x = 0.19;
	quality_static_pose.position.y = 3.25;
	quality_static_pose.position.z = 1.15;
	quality_static_pose.orientation = fixed_orientation_;
}

RobotController::~RobotController() {}

bool RobotController::Planner() {
	ROS_INFO_STREAM("Planning started...");
	if (robot_move_group_.plan(robot_planner_) ==
			moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		plan_success_ = true;
		ROS_INFO_STREAM("Planner succeeded!");
	} else {
		plan_success_ = false;
		ROS_WARN_STREAM("Planner failed!");
	}

	return plan_success_;
}

void RobotController::collisionAvoidance(){

	 collision_object.header.frame_id = robot_move_group_.getPlanningFrame();

	 // The id of the object is used to identify it.
	 collision_object.id = "box1";

	 // Define a box to add to the world.
	 shape_msgs::SolidPrimitive primitive;
	 primitive.type = primitive.BOX;
	 primitive.dimensions.resize(3);
	 primitive.dimensions[0] = 0.1;
	 primitive.dimensions[1] = 0.1;
	 primitive.dimensions[2] = 0.1;

	 // Define a pose for the box (specified relative to frame_id)
	 geometry_msgs::Pose box_pose;
	 box_pose.orientation.w = 1.0;
	 box_pose.position.x = -0.30;
	 box_pose.position.y = -0.38;
	 box_pose.position.z = 1.2;

	 collision_object.primitives.push_back(primitive);
	 collision_object.primitive_poses.push_back(box_pose);
	 collision_object.operation = collision_object.ADD;

//	 std::vector<moveit_msgs::CollisionObject> collision_objects;
	 collision_objects.push_back(collision_object);

	 // Now, let's add the collision object into the world
	 ROS_INFO_NAMED("tutorial", "Add an object into the world");
	 planning_scene_interface.addCollisionObjects(collision_objects);

}



void RobotController::Execute() {
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}
}

void RobotController::GoToBinStaticPosition() {
	moveToTarget(static_bin_pose);
	ros::Duration(1.0).sleep();
	ROS_INFO_STREAM("At Bin safe Home position");
}

void RobotController::moveToTarget(geometry_msgs::Pose final_pose) {
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose dt_pose;
	robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
			ros::Duration(10));
	robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
			robot_tf_transform_);


	current_pose_.position.x = robot_tf_transform_.getOrigin().x();
	current_pose_.position.y = robot_tf_transform_.getOrigin().y();
	current_pose_.position.z = robot_tf_transform_.getOrigin().z();
	current_pose_.orientation.x = robot_tf_transform_.getRotation().x();
	current_pose_.orientation.y = robot_tf_transform_.getRotation().y();
	current_pose_.orientation.z = robot_tf_transform_.getRotation().z();
	current_pose_.orientation.w = robot_tf_transform_.getRotation().w();

	dt_pose.position.x =
			(final_pose.position.x - current_pose_.position.x) / 10;
	dt_pose.position.y =
			(final_pose.position.y - current_pose_.position.y) / 10;
	dt_pose.position.z =
			(final_pose.position.z - current_pose_.position.z) / 10;
	dt_pose.orientation.x = 0;
	dt_pose.orientation.y = 0;
	dt_pose.orientation.z = 0;
	dt_pose.orientation.w = 0;

	geometry_msgs::Pose next_pose;
	for (int i = 1; i <= 10; i++) {
		next_pose.position.x = current_pose_.position.x +
				i * dt_pose.position.x;
		next_pose.position.y = current_pose_.position.y +
				i * dt_pose.position.y;
		next_pose.position.z = current_pose_.position.z +
				i * dt_pose.position.z;
		next_pose.orientation.x = current_pose_.orientation.x +
				i * dt_pose.orientation.x;
		next_pose.orientation.y = current_pose_.orientation.y +
				i * dt_pose.orientation.y;
		next_pose.orientation.z = current_pose_.orientation.z +
				i * dt_pose.orientation.z;
		next_pose.orientation.w = current_pose_.orientation.w +
				i * dt_pose.orientation.w;
		GoToTarget(next_pose);
		ros::Duration(1.0).sleep();
	}
	GoToTarget(final_pose);
	ros::Duration(1.0).sleep();
//	GoToTarget(waypoints);
}

void RobotController::moveToTargetinPieces(geometry_msgs::Pose final_pose) {
	std::vector<geometry_msgs::Pose> waypoints;
		geometry_msgs::Pose dt_pose;
		robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
				ros::Duration(10));
		robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
				robot_tf_transform_);


		current_pose_.position.x = robot_tf_transform_.getOrigin().x();
		current_pose_.position.y = robot_tf_transform_.getOrigin().y();
		current_pose_.position.z = robot_tf_transform_.getOrigin().z();
		current_pose_.orientation.x = robot_tf_transform_.getRotation().x();
		current_pose_.orientation.y = robot_tf_transform_.getRotation().y();
		current_pose_.orientation.z = robot_tf_transform_.getRotation().z();
		current_pose_.orientation.w = robot_tf_transform_.getRotation().w();

		dt_pose.position.x =
				(final_pose.position.x - current_pose_.position.x) / 10;
		dt_pose.position.y =
				(final_pose.position.y - current_pose_.position.y) / 10;
		dt_pose.position.z =
				(final_pose.position.z - current_pose_.position.z) / 10;
		dt_pose.orientation.x = 0;
		dt_pose.orientation.y = 0;
		dt_pose.orientation.z = 0;
		dt_pose.orientation.w = 0;
		double dpara = -0.1;

		geometry_msgs::Pose next_pose;
		for (int i,j = 1; i <= 10; i++,j++) {
			if(i >= 6) {
				j = 10 - i;
			}
//			dt_pose.position.x-=dpara;
			next_pose.position.x = current_pose_.position.x +
					i * dt_pose.position.x + j*dpara;
			next_pose.position.y = current_pose_.position.y +
					i * dt_pose.position.y;
			next_pose.position.z = current_pose_.position.z +
					i * dt_pose.position.z;
//			next_pose.orientation.x = current_pose_.orientation.x +
//					i * dt_pose.orientation.x;
//			next_pose.orientation.y = current_pose_.orientation.y +
//					i * dt_pose.orientation.y;
//			next_pose.orientation.z = current_pose_.orientation.z +
//					i * dt_pose.orientation.z;
//			next_pose.orientation.w = current_pose_.orientation.w +
//					i * dt_pose.orientation.w;
//			waypoints.emplace_back(next_pose);
			GoToTarget(next_pose);
			ros::Duration(1.0).sleep();
		}
		GoToTarget(final_pose);
		ros::Duration(1.0).sleep();

}
void RobotController::GoToTarget(
		std::vector<geometry_msgs::Pose> waypoints) {
	ros::AsyncSpinner spinner(4);
	spinner.start();

	for (auto i : waypoints) {
		i.orientation.x = fixed_orientation_.x;
		i.orientation.y = fixed_orientation_.y;
		i.orientation.z = fixed_orientation_.z;
		i.orientation.w = fixed_orientation_.w;
	}

	moveit_msgs::RobotTrajectory traj;
	auto fraction =
			robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

	ROS_WARN_STREAM("Fraction: " << fraction * 100);
	ros::Duration(0.05).sleep();

	robot_planner_.trajectory_ = traj;

	//if (fraction >= 0.3) {
	robot_move_group_.execute(robot_planner_);
	ros::Duration(0.05).sleep();
	//    } else {
	//        ROS_ERROR_STREAM("Safe Trajectory not found!");
	//    }
}

void RobotController::GoToTarget(
		std::initializer_list<geometry_msgs::Pose> list) {
	ros::AsyncSpinner spinner(4);
	spinner.start();

	std::vector<geometry_msgs::Pose> waypoints;
	for (auto i : list) {
		i.orientation.x = fixed_orientation_.x;
		i.orientation.y = fixed_orientation_.y;
		i.orientation.z = fixed_orientation_.z;
		i.orientation.w = fixed_orientation_.w;
		waypoints.emplace_back(i);
	}

	moveit_msgs::RobotTrajectory traj;
	auto fraction =
			robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

	ROS_WARN_STREAM("Fraction: " << fraction * 100);
	ros::Duration(0.05).sleep();

	robot_planner_.trajectory_ = traj;

	//if (fraction >= 0.3) {
	robot_move_group_.execute(robot_planner_);
	ros::Duration(0.05).sleep();
	//    } else {
	//        ROS_ERROR_STREAM("Safe Trajectory not found!");
	//    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
	target_pose_.orientation = fixed_orientation_;
	target_pose_.position = pose.position;
	ros::AsyncSpinner spinner(4);
	robot_move_group_.setPoseTarget(target_pose_);
	spinner.start();
	if (this->Planner()) {
		ROS_INFO_STREAM("Point success");
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}
	ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToAGV(const geometry_msgs::Pose& pose) {
	ROS_WARN_STREAM("placing in AGV HAHAHAHA	");
	ros::AsyncSpinner spinner(4);
	//	pose.position.z += 0.1;
	robot_move_group_.setPoseTarget(pose);
	spinner.start();
	if (this->Planner()) {
		ROS_INFO_STREAM("Point success");
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}
	ROS_INFO_STREAM("Point reached...");
}


void RobotController::GripperToggle(const bool& state) {
	gripper_service_.request.enable = state;
	gripper_client_.call(gripper_service_);
	ros::Duration(0.01).sleep();
	// if (gripper_client_.call(gripper_service_)) {
	if (gripper_service_.response.success) {
		ROS_INFO_STREAM("Gripper activated!");
	} else {
		ROS_WARN_STREAM("Gripper activation failed!");
	}
}

void RobotController::GripperCallback(
		const osrf_gear::VacuumGripperState::ConstPtr& grip) {
	gripper_state_ = grip->attached;
}



bool RobotController::isPartAttached(){
	return gripper_state_;
}

bool RobotController::isAtQualitySensor() {
	return is_at_qualitySensor;
}

void RobotController::setAtQualitySensor(){
	is_at_qualitySensor = true;
}

void RobotController::GoToPose(const std::vector<double> & pose ) {
	robot_move_group_.setJointValueTarget(pose);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}

	ros::Duration(0.05).sleep();

}

geometry_msgs::Pose RobotController::getHomeCartPose(){
	return home_cart_pose_;
}

void RobotController::SendRobotHome() {
	// ros::Duration(2.0).sleep();
	robot_move_group_.setJointValueTarget(home_joint_pose_);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.05).sleep();
	}

	ros::Duration(0.05).sleep();
}

void RobotController::dropInTrash(){
	// ros::Duration(2.0).sleep();
	robot_move_group_.setJointValueTarget(trash_bin_joint_position_);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.05).sleep();
	}
	GripperToggle(false);
	ros::Duration(0.05).sleep();
}

void RobotController::GoToQualityCamera(){
	// ros::Duration(2.0).sleep();
	robot_move_group_.setJointValueTarget(quality_cam_joint_position_);
	// this->execute();
	//		ros::AsyncSpinner spinner(4);
	//		spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.05).sleep();
	}
	is_at_qualitySensor = true;
	ros::Duration(0.05).sleep();
}

void RobotController::GoToQualityCameraFromBin(){
	// ros::Duration(2.0).sleep();
	moveToTargetinPieces(quality_static_pose);
	is_at_qualitySensor = true;
	// this->execute();
	//		ros::AsyncSpinner spinner(4);
	//		spinner.start();
	ros::Duration(1.0).sleep();
	ROS_INFO_STREAM("At quality camera check position");
}
