//
// Created by zeid on 3/1/20.
//

#include <ariac_order_part.h>

AriacOrderPart::AriacOrderPart():part_type_{}, part_frame_{}, num_parts(1){};
AriacOrderPart::~AriacOrderPart(){};

void AriacOrderPart::set_part_type(std::string part_type){
    part_type_ = part_type;
};

// void AriacOrderPart::set_part_frame(int part_frame){
//     part_frame_ = part_frame;
// };


void AriacOrderPart::set_part_pose(geometry_msgs::Pose part_pose){
    part_pose_ = part_pose;
};

const std::string AriacOrderPart::get_part_type(){
    return part_type_;
};

// const int AriacOrderPart::get_part_frame(){
//     return part_frame_;
// };

int AriacOrderPart::get_num_parts(){
    return num_parts;
}

void AriacOrderPart::increment(){
    num_parts += 1;
}

void AriacOrderPart::decrement(){
    num_parts -= 1;
}

const geometry_msgs::Pose AriacOrderPart::get_part_pose(){
    return part_pose_ ;
};
