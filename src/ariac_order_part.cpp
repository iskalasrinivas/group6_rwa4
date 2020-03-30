/**
 * @file      src/ariac_order_part.cpp
 * @brief     Header file for building map
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

#include <ariac_order_part.h>

AriacOrderPart::AriacOrderPart(std::string part_type, geometry_msgs::Pose t_pose):part_type_(part_type), tray_pose_(t_pose) {
	worldTransformation();
}
AriacOrderPart::~AriacOrderPart() {}

void AriacOrderPart::setPartType(std::string part_type) {
	part_type_ = part_type;
}

void AriacOrderPart::setCurrentPose(geometry_msgs::Pose pose) {
	current_pose_ = pose;
}

const std::string AriacOrderPart::getPartType() {
	return part_type_;
}

const geometry_msgs::Pose AriacOrderPart::getEndPose() {
	return end_pose_;
}

const geometry_msgs::Pose AriacOrderPart::getCurrentPose() {
	return current_pose_;
}

void AriacOrderPart::worldTransformation(){
	auto current_time = ros::Time::now();
	tf2_ros::TransformListener tfListener(tfBuffer);

	tS_b_p.header.stamp = current_time;
	tS_b_p.header.frame_id = "kit_tray_1";
	tS_b_p.child_frame_id = "tray_bin_child";
	tS_b_p.transform.translation.x = tray_pose_.position.x;
	tS_b_p.transform.translation.y = tray_pose_.position.y;
	tS_b_p.transform.translation.z = tray_pose_.position.z;
	tS_b_p.transform.rotation.x = tray_pose_.orientation.x;
	tS_b_p.transform.rotation.y = tray_pose_.orientation.y;
	tS_b_p.transform.rotation.z = tray_pose_.orientation.z;
	tS_b_p.transform.rotation.w = tray_pose_.orientation.w;
	br_s_c.sendTransform(tS_b_p);
	ros::Duration(0.001).sleep();
	try {
		tS_w_p = tfBuffer.lookupTransform
				("world", "tray_bin_child",
						ros::Time(0));
	    end_pose_.position.x = tS_w_p.transform.translation.x;
	    end_pose_.position.y = tS_w_p.transform.translation.y;
	    end_pose_.position.z = tS_w_p.transform.translation.z;
	    end_pose_.orientation.x = tS_w_p.transform.rotation.x;
	    end_pose_.orientation.y = tS_w_p.transform.rotation.y;
	    end_pose_.orientation.z = tS_w_p.transform.rotation.z;
	    end_pose_.orientation.w = tS_w_p.transform.rotation.w;

	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("exception");
		ROS_WARN("%s", ex.what());
		ros::Duration(0.001).sleep();
	}

}

