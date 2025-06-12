// ================================
// odom_to_tf_with_laser_transform.hpp
// ================================

#ifndef ODOM_TO_TF_WITH_LASER_TRANSFORM_HPP
#define ODOM_TO_TF_WITH_LASER_TRANSFORM_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <cstring>
#include <algorithm>
#include "occupancy_map.h"

class OdomToTFWithLaserTransform : public rclcpp::Node
{
public:
    OdomToTFWithLaserTransform();

private:
    // Callback functions
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Parameter callback
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // Transformation functions
    void transform_laser_to_map(const sensor_msgs::msg::LaserScan::SharedPtr& laser_msg,
                               const geometry_msgs::msg::Pose& robot_pose,
                               Eigen::MatrixXf& map_points);
    
    
    // Publishing functions
    void publish_pointcloud(const Eigen::MatrixXf& points, const rclcpp::Time& timestamp);
    void publish_markers(const Eigen::MatrixXf& points, const rclcpp::Time& timestamp);
    
    // Utility functions
    void declare_and_get_parameters();
    Eigen::Affine2f get_robot_transform(const geometry_msgs::msg::Pose& robot_pose);
    
    // ROS2 components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // State variables
    geometry_msgs::msg::Pose robot_pose_;
    bool robot_pose_available_;
    rclcpp::Time last_log_time_;
    
    // Parameters
    struct Parameters {
        std::string odom_topic;
        std::string laser_topic;
        std::string map_frame;
        std::string laser_frame;
        std::string base_frame;
        double laser_offset_x;
        double laser_offset_y;
        double laser_offset_yaw;
        double min_filter_distance;
        double max_range_filter;
        double log_throttle_duration;
        double marker_size;
        std::vector<double> marker_color;
        std::vector<double> target_area; // [x_min, x_max, y_min, y_max]
        double map_resolution; // Resolution of the occupancy map
        double prob_threshold; // Probability threshold for occupancy map
    } params_;

    // Occupancy map
    std::shared_ptr<OccupancyMap> occupancy_map_;
};

#endif // ODOM_TO_TF_WITH_LASER_TRANSFORM_HPP