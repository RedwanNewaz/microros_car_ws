#include "pirobot_mapping/odom_to_tf_with_laser_transform.h"

OdomToTFWithLaserTransform::OdomToTFWithLaserTransform() 
    : Node("odom_to_tf_with_laser"), robot_pose_available_(false)
{
    // Declare and get parameters
    declare_and_get_parameters();
    
    // Setup parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&OdomToTFWithLaserTransform::parameters_callback, this, std::placeholders::_1));
    
    // Create subscriptions
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        params_.odom_topic,
        10,
        std::bind(&OdomToTFWithLaserTransform::odom_callback, this, std::placeholders::_1)
    );
    
    laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        params_.laser_topic,
        10,
        std::bind(&OdomToTFWithLaserTransform::laser_callback, this, std::placeholders::_1)
    );

    // Initialize occupancy map
    occupancy_map_ = std::make_shared<OccupancyMap>(params_.target_area, params_.map_resolution);
    
    // Create publishers and broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/laser_points_markers", 10);
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_points_cloud", 10);
    
    // Initialize timing
    last_log_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "OdomToTFWithLaserTransform node initialized");
}

void OdomToTFWithLaserTransform::declare_and_get_parameters()
{
    // Declare parameters with default values and descriptions
    this->declare_parameter("odom_topic", "/vicon/pirobot/pirobot", 
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Odometry topic name"));
    
    this->declare_parameter("laser_topic", "/scan",
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Laser scan topic name"));
    
    this->declare_parameter("map_frame", "map",
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Map frame ID"));
    
    this->declare_parameter("laser_frame", "laser_frame",
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Laser frame ID"));
    
    this->declare_parameter("base_frame", "base_link",
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Base frame ID for visualization"));
    
    this->declare_parameter("laser_offset_x", 0.0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Laser X offset from robot center (meters)"));
    
    this->declare_parameter("laser_offset_y", 0.0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Laser Y offset from robot center (meters)"));
    
    this->declare_parameter("laser_offset_yaw", 0.125,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Laser yaw offset from robot orientation (radians)"));
    
    this->declare_parameter("min_filter_distance", 0.25,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Minimum distance to filter points near robot (meters)"));
    
    this->declare_parameter("max_range_filter", 10.0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Maximum range to consider for filtering (meters)"));
    
    this->declare_parameter("log_throttle_duration", 2.0,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Log throttle duration in seconds"));
    
    this->declare_parameter("marker_size", 0.05,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Marker visualization size"));
    
    this->declare_parameter("marker_color", std::vector<double>{1.0, 0.0, 0.0, 0.8},
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Marker color [r, g, b, a]"));
    this->declare_parameter("target_area", std::vector<double>{-6.0, 4.0, -3.0, 3.0},
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Target area for laser points [x_min, x_max, y_min, y_max]"));
    this->declare_parameter("map_resolution", 0.05,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Resolution of the occupancy map (meters)"));
    this->declare_parameter("prob_threshold", 0.33,
        rcl_interfaces::msg::ParameterDescriptor()
            .set__description("Probability threshold for occupancy map (0.0 to 1.0)"));
    
    // Get parameter values
    params_.odom_topic = this->get_parameter("odom_topic").as_string();
    params_.laser_topic = this->get_parameter("laser_topic").as_string();
    params_.map_frame = this->get_parameter("map_frame").as_string();
    params_.laser_frame = this->get_parameter("laser_frame").as_string();
    params_.base_frame = this->get_parameter("base_frame").as_string();
    params_.laser_offset_x = this->get_parameter("laser_offset_x").as_double();
    params_.laser_offset_y = this->get_parameter("laser_offset_y").as_double();
    params_.laser_offset_yaw = this->get_parameter("laser_offset_yaw").as_double();
    params_.min_filter_distance = this->get_parameter("min_filter_distance").as_double();
    params_.max_range_filter = this->get_parameter("max_range_filter").as_double();
    params_.log_throttle_duration = this->get_parameter("log_throttle_duration").as_double();
    params_.marker_size = this->get_parameter("marker_size").as_double();
    params_.marker_color = this->get_parameter("marker_color").as_double_array();
    params_.target_area = this->get_parameter("target_area").as_double_array();
    params_.map_resolution = this->get_parameter("map_resolution").as_double();
    params_.prob_threshold = this->get_parameter("prob_threshold").as_double();
}

rcl_interfaces::msg::SetParametersResult OdomToTFWithLaserTransform::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "laser_offset_x") {
            params_.laser_offset_x = param.as_double();
        } else if (param.get_name() == "laser_offset_y") {
            params_.laser_offset_y = param.as_double();
        } else if (param.get_name() == "laser_offset_yaw") {
            params_.laser_offset_yaw = param.as_double();
        } else if (param.get_name() == "min_filter_distance") {
            params_.min_filter_distance = param.as_double();
        } else if (param.get_name() == "max_range_filter") {
            params_.max_range_filter = param.as_double();
        } else if (param.get_name() == "marker_size") {
            params_.marker_size = param.as_double();
        } else if (param.get_name() == "marker_color") {
            params_.marker_color = param.as_double_array();
        }
    }
    
    return result;
}

void OdomToTFWithLaserTransform::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Broadcast transform from odometry data
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = params_.map_frame;
    transform.child_frame_id = params_.laser_frame;
    
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(transform);
    
    // Store robot pose
    robot_pose_ = msg->pose.pose;
    robot_pose_available_ = true;
    
    // Throttled logging
    auto now = this->get_clock()->now();
    if ((now - last_log_time_).seconds() >= params_.log_throttle_duration) {
        // RCLCPP_INFO(this->get_logger(), "Published TF: %s -> %s", 
        //            transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
        last_log_time_ = now;
    }
}

void OdomToTFWithLaserTransform::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!robot_pose_available_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "No robot pose available, skipping laser processing");
        return;
    }
    
    try {
        // Transform laser points to map frame using Eigen
        Eigen::MatrixXf scan_points;
        transform_laser_to_map(msg, robot_pose_, scan_points);
        
        if (scan_points.cols() == 0) {
            return;
        }
    
        
        if (scan_points.cols() > 0) {
            auto timestamp = this->get_clock()->now();

            // populate occupancy map
            for (int i = 0; i < scan_points.cols(); ++i) {
                occupancy_map_->insert(scan_points(0, i), scan_points(1, i));
            }

            auto map_data = occupancy_map_->getMap(params_.prob_threshold);
            // Get the number of points (columns in the Eigen matrix)
            size_t num_points = map_data.size();

            // Create an Eigen::MatrixXf with 3 rows (for X, Y, Z) and num_points columns
            // Eigen::MatrixXf is a dynamic-size matrix of floats.
            // We initialize it with zeros to ensure all elements have a default value.
            Eigen::MatrixXf map_points(3, num_points);

            // Iterate through the cloud_data vector
            for (size_t i = 0; i < num_points; ++i) {
                // Access the current std::array (representing one point)
                const std::array<double, 3>& point = map_data[i];

                // Assign the X, Y, Z components to the corresponding row and column
                // map_points(row_index, column_index)
                map_points(0, i) = static_cast<float>(point[0]); // X data
                map_points(1, i) = static_cast<float>(point[1]); // Y data
                map_points(2, i) = static_cast<float>(point[2]); // Z data
            }

            
            // Publish visualizations
            publish_pointcloud(map_points, timestamp);
            publish_markers(scan_points, timestamp);
            
            // Throttled logging
            auto now = this->get_clock()->now();
            if ((now - last_log_time_).seconds() >= params_.log_throttle_duration) {
                RCLCPP_INFO(this->get_logger(), "Published %ld laser points to map frame", 
                scan_points.cols());
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in laser transformation: %s", e.what());
    }
}

void OdomToTFWithLaserTransform::transform_laser_to_map(
    const sensor_msgs::msg::LaserScan::SharedPtr& laser_msg,
    const geometry_msgs::msg::Pose& robot_pose,
    Eigen::MatrixXf& map_points)
{
    // Filter valid ranges
    std::vector<float> valid_ranges;
    std::vector<float> valid_angles;
    
    for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
        float range = laser_msg->ranges[i];
        if (range >= laser_msg->range_min +  params_.min_filter_distance && 
            range <= std::min(laser_msg->range_max, static_cast<float>(params_.max_range_filter)) && 
            std::isfinite(range)) {
            valid_ranges.push_back(range);
            float angle = laser_msg->angle_min + i * laser_msg->angle_increment;
            valid_angles.push_back(angle);
        }
    }
    
    if (valid_ranges.empty()) {
        map_points = Eigen::MatrixXf(2, 0);
        return;
    }
    
    const size_t num_points = valid_ranges.size();
    
    // Create Eigen matrices for efficient computation
    Eigen::VectorXf ranges = Eigen::Map<Eigen::VectorXf>(valid_ranges.data(), num_points);
    Eigen::VectorXf angles = Eigen::Map<Eigen::VectorXf>(valid_angles.data(), num_points);
    
    // Add laser offset yaw to angles
    angles.array() += params_.laser_offset_yaw;
    
    // Convert to cartesian in laser frame using vectorized operations
    Eigen::MatrixXf laser_points(2, num_points);
    laser_points.row(0) = ranges.array() * angles.array().cos() + params_.laser_offset_x;  // X
    laser_points.row(1) = ranges.array() * angles.array().sin() + params_.laser_offset_y;  // Y
    
    // Get robot pose parameters
    float robot_x = robot_pose.position.x;
    float robot_y = robot_pose.position.y;
    
    // Convert robot quaternion to yaw angle
    tf2::Quaternion quat;
    tf2::fromMsg(robot_pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float robot_yaw = static_cast<float>(yaw);
    
    // Create rotation matrix components
    float cos_yaw = std::cos(robot_yaw);
    float sin_yaw = std::sin(robot_yaw);
    
    // Transform all points at once using vectorized operations
    map_points = Eigen::MatrixXf(2, num_points);
    map_points.row(0) = (laser_points.row(0) * cos_yaw - laser_points.row(1) * sin_yaw).array() + robot_x;
    map_points.row(1) = (laser_points.row(0) * sin_yaw + laser_points.row(1) * cos_yaw).array() + robot_y;
}

void OdomToTFWithLaserTransform::publish_pointcloud(
    const Eigen::MatrixXf& points, 
    const rclcpp::Time& timestamp)
{
    if (points.cols() == 0) return;
    
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header.stamp = timestamp;
    cloud_msg->header.frame_id = params_.base_frame;
    cloud_msg->height = 1;
    cloud_msg->width = points.cols();
    
    // Define point cloud fields
    cloud_msg->fields.resize(3);
    cloud_msg->fields[0].name = "x";
    cloud_msg->fields[0].offset = 0;
    cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg->fields[0].count = 1;
    
    cloud_msg->fields[1].name = "y";
    cloud_msg->fields[1].offset = 4;
    cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg->fields[1].count = 1;
    
    cloud_msg->fields[2].name = "z";
    cloud_msg->fields[2].offset = 8;
    cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg->fields[2].count = 1;
    
    cloud_msg->is_bigendian = false;
    cloud_msg->point_step = 12;
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
    
    // Pack point data efficiently
    cloud_msg->data.resize(cloud_msg->row_step);
    float* data_ptr = reinterpret_cast<float*>(cloud_msg->data.data());
    
    for (int i = 0; i < points.cols(); ++i) {
        data_ptr[i * 3] = points(0, i);     // x
        data_ptr[i * 3 + 1] = points(1, i); // y
        data_ptr[i * 3 + 2] = 0.0f;         // z
    }
    
    pointcloud_pub_->publish(std::move(cloud_msg));
}

void OdomToTFWithLaserTransform::publish_markers(
    const Eigen::MatrixXf& points,
    const rclcpp::Time& timestamp)
{
    if (points.cols() == 0) return;
    
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.base_frame;
    marker.header.stamp = timestamp;
    marker.ns = "laser_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set marker properties from parameters
    marker.scale.x = params_.marker_size;
    marker.scale.y = params_.marker_size;
    
    if (params_.marker_color.size() >= 4) {
        marker.color.r = params_.marker_color[0];
        marker.color.g = params_.marker_color[1];
        marker.color.b = params_.marker_color[2];
        marker.color.a = params_.marker_color[3];
    } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
    }
    
    // Add points efficiently
    marker.points.reserve(points.cols());
    for (int i = 0; i < points.cols(); ++i) {
        geometry_msgs::msg::Point point;
        point.x = points(0, i);
        point.y = points(1, i);
        point.z = 0.0;
        marker.points.push_back(point);
    }
    
    marker_array->markers.push_back(marker);
    marker_pub_->publish(std::move(marker_array));
}
