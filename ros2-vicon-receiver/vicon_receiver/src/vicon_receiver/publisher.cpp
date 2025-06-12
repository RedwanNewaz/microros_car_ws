#include "vicon_receiver/publisher.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node)
{
    position_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(const PositionStruct& p)
{
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    // convert the position struct to a ROS2 message nav_msgs::msg::Odometry
 
    msg->header.frame_id = "world";
    msg->child_frame_id = p.segment_name;
    msg->pose.pose.position.x = p.translation[0] / 1.0e3;
    msg->pose.pose.position.y = p.translation[1] / 1.0e3;
    msg->pose.pose.position.z = p.translation[2] / 1.0e3;
    msg->pose.pose.orientation.x = p.rotation[0];
    msg->pose.pose.orientation.y = p.rotation[1];
    msg->pose.pose.orientation.z = p.rotation[2];
    msg->pose.pose.orientation.w = p.rotation[3];

    // msg->x_trans = p.translation[0];
    // msg->y_trans = p.translation[1];
    // msg->z_trans = p.translation[2];
    // msg->x_rot = p.rotation[0];
    // msg->y_rot = p.rotation[1];
    // msg->z_rot = p.rotation[2];
    // msg->w = p.rotation[3];
    // msg->subject_name = p.subject_name;
    // msg->segment_name = p.segment_name;
    // msg->frame_number = p.frame_number;
    // msg->translation_type = p.translation_type;
    position_publisher_->publish(*msg);
}
