#include "bow/planner_interface.h"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

BowPlannerInterface::BowPlannerInterface()
    : Node("bow_planner_interface")
{
    // Initialize state variables
    current_state_.setZero();
    goal_state_.setZero();
    last_pointcloud_time_ = this->get_clock()->now();
    
    // Initialize components
    initializeParameters();
    setupPublishersAndSubscribers();
    
    // // Initialize collision checker with empty data initially
    // std::vector<double> empty_x, empty_y;
    // collision_checker_ = std::make_shared<bow::CollisionChecker>(empty_x, empty_y, params_.robot_radius);
    
    setupTimer();
    
    RCLCPP_INFO(this->get_logger(), "BOW Planner Interface initialized successfully");
    logStatus();
}

void BowPlannerInterface::initializeParameters()
{
    // Declare and get parameters with defaults
    this->declare_parameter("dt", params_.dt);
    this->declare_parameter("predict_time", params_.predict_time);
    this->declare_parameter("robot_radius", params_.robot_radius);
    this->declare_parameter("goal_radius", params_.goal_radius);
    this->declare_parameter("pref_speed_index", params_.pref_speed_index);
    this->declare_parameter("num_samples", params_.num_samples);
    this->declare_parameter("max_speed", params_.max_speed);
    this->declare_parameter("min_speed", params_.min_speed);
    this->declare_parameter("max_yawrate", params_.max_yawrate);
    this->declare_parameter("map_resolution", params_.map_resolution);
    this->declare_parameter("position_tolerance", params_.position_tolerance);
    this->declare_parameter("angle_tolerance", params_.angle_tolerance);
    this->declare_parameter("max_planning_iterations", params_.max_planning_iterations);
    
    // Get parameter values
    params_.dt = this->get_parameter("dt").as_double();
    params_.predict_time = this->get_parameter("predict_time").as_double();
    params_.robot_radius = this->get_parameter("robot_radius").as_double();
    params_.goal_radius = this->get_parameter("goal_radius").as_double();
    params_.pref_speed_index = this->get_parameter("pref_speed_index").as_int();
    params_.num_samples = this->get_parameter("num_samples").as_int();
    params_.max_speed = this->get_parameter("max_speed").as_double();
    params_.min_speed = this->get_parameter("min_speed").as_double();
    params_.max_yawrate = this->get_parameter("max_yawrate").as_double();
    params_.map_resolution = this->get_parameter("map_resolution").as_double();
    params_.position_tolerance = this->get_parameter("position_tolerance").as_double();
    params_.angle_tolerance = this->get_parameter("angle_tolerance").as_double();
    params_.max_planning_iterations = this->get_parameter("max_planning_iterations").as_int();
    params_.boundary = this->get_parameter("boundary").as_double_array();
    
    // Validate parameters
    if (params_.dt <= 0.0 || params_.dt > 1.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid dt parameter: %f. Using default: 0.2", params_.dt);
        params_.dt = 0.2;
    }
    
    if (params_.robot_radius <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid robot_radius: %f. Using default: 0.25", params_.robot_radius);
        params_.robot_radius = 0.25;
    }
    
    // Create parameter manager for BOW planner
    pm_ = std::make_shared<param_manager>();
    pm_->dt = params_.dt;
    pm_->predict_time = params_.predict_time;
    pm_->robot_radius = params_.robot_radius;
    pm_->goal_radius = params_.goal_radius;
    pm_->pref_speed_index = params_.pref_speed_index;
    pm_->num_samples = params_.num_samples;
    pm_->max_speed = params_.max_speed;
    pm_->min_speed = params_.min_speed;
    pm_->max_yawrate = params_.max_yawrate;
    pm_->map_resolution = params_.map_resolution;
    pm_->boundary = params_.boundary;
}

void BowPlannerInterface::setupPublishersAndSubscribers()
{
    // Quality of Service settings
    auto qos_reliable = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    auto qos_best_effort = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    // Subscribers
    vicon_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vicon/pirobot/pirobot", qos_best_effort,
        std::bind(&BowPlannerInterface::viconCallback, this, _1));
    
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/laser_points_cloud", qos_best_effort,
        std::bind(&BowPlannerInterface::pointCloudCallback, this, _1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", qos_reliable,
        std::bind(&BowPlannerInterface::goalCallback, this, _1));
    
    // Publishers
    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", qos_reliable);
    pose_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_pose", qos_reliable);
    obstacles_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", qos_reliable);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_reliable);
}

void BowPlannerInterface::setupTimer()
{
    auto timer_period = std::chrono::milliseconds(static_cast<int>(params_.dt * 1000));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&BowPlannerInterface::controlTimerCallback, this));
}

void BowPlannerInterface::viconCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    try {
        // Extract position and orientation
        const auto& position = msg->pose.pose.position;
        const auto& orientation = msg->pose.pose.orientation;
        
        // Convert quaternion to yaw
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Normalize yaw angle
        yaw = normalizeAngle(yaw);
        
        // Update current state with thread safety
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_(0, 0) = position.x;
            current_state_(1, 0) = position.y;
            current_state_(2, 0) = yaw;
            
            // Extract velocities if available
            if (msg->twist.twist.linear.x != 0.0 || msg->twist.twist.angular.z != 0.0) {
                current_state_(3, 0) = msg->twist.twist.linear.x;
                current_state_(4, 0) = msg->twist.twist.angular.z;
            }
        }
        
        // Validate state
        if (!validateState(current_state_)) {
            RCLCPP_WARN(this->get_logger(), "Invalid state received from Vicon");
            return;
        }
        
        publishPose(current_state_);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in vicon callback: %s", e.what());
    }
}

void BowPlannerInterface::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try {
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        
        if (pcl_cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }
        
        // Extract points for collision checker
        std::vector<double> x_coords, y_coords;
        std::vector<geometry_msgs::msg::Point> viz_points;
        
        x_coords.reserve(pcl_cloud.size());
        y_coords.reserve(pcl_cloud.size());
        viz_points.reserve(pcl_cloud.size());
        
        for (const auto& point : pcl_cloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y)) {
                x_coords.push_back(point.x);
                y_coords.push_back(point.y);
                
                geometry_msgs::msg::Point viz_point;
                viz_point.x = point.x;
                viz_point.y = point.y;
                viz_point.z = 0.0;
                viz_points.push_back(viz_point);
            }
        }
        
        // Update collision checker with thread safety
        {
            std::lock_guard<std::mutex> lock(collision_checker_mutex_);
            //FIXME: arguments for collision checker constructor
            collision_checker_ = std::make_shared<bow::CollisionChecker>(
                x_coords, y_coords, params_.robot_radius, params_.map_resolution, params_.boundary);
        }
        
        // Publish obstacle visualization
        publishObstacles(viz_points);
        
        last_pointcloud_time_ = this->get_clock()->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Updated collision checker with %zu points", x_coords.size());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

void BowPlannerInterface::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    try {
        // Update goal state
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            goal_state_(0, 0) = msg->pose.position.x;
            goal_state_(1, 0) = msg->pose.position.y;
        }
        
        initialized_ = true;
        goal_reached_ = false;
        use_cached_trajectory_ = false;
        
        RCLCPP_INFO(this->get_logger(), "New goal set: (%.2f, %.2f)", 
                   goal_state_(0, 0), goal_state_(1, 0));
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in goal callback: %s", e.what());
    }
}

void BowPlannerInterface::controlTimerCallback()
{
    if (!initialized_ || goal_reached_) {
        return;
    }
    
    try {
        // Check if goal is reached
        if (isGoalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
            stopRobot();
            goal_reached_ = true;
            return;
        }
        
        // Plan trajectory periodically or when needed
        bool should_replan = (loop_count_ % params_.pref_speed_index == 0) || !use_cached_trajectory_;
        
        std::vector<bow::State> trajectory;
        
        if (should_replan) {
            auto [success, new_trajectory] = planTrajectory();
            if (success && !new_trajectory.empty() ) {

                if(collision_checker_->isCollision(new_trajectory)) {
                    RCLCPP_WARN(this->get_logger(), "Collision detected in planned trajectory, retrying");
                    use_cached_trajectory_ = false;
                    return;
                }
                last_trajectory_ = std::move(new_trajectory);
                use_cached_trajectory_ = true;
                trajectory = last_trajectory_;
                publishTrajectory(trajectory);
                RCLCPP_DEBUG(this->get_logger(), "New trajectory planned with %zu states", trajectory.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "Planning failed, using cached trajectory or stopping");
                if (!last_trajectory_.empty()) {
                    trajectory = last_trajectory_;
                } else {
                    stopRobot();
                    return;
                }
            }
        } else {
            trajectory = last_trajectory_;
        }
        
        // Extract and publish control commands
        if (!trajectory.empty()) {
            auto [linear_vel, angular_vel] = extractControlCommands(trajectory);
            publishCmdVel(linear_vel, angular_vel);
            
            // Update current state velocities
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_state_(3, 0) = linear_vel;
                current_state_(4, 0) = angular_vel;
            }
        }
        
        loop_count_++;
        
        // Periodic status logging
        if (loop_count_ % 50 == 0) {
            logStatus();
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in control loop: %s", e.what());
        stopRobot();
    }
}

std::pair<bool, std::vector<bow::State>> BowPlannerInterface::planTrajectory()
{
    bow::BOPlanner planner(current_state_, goal_state_, 
        collision_checker_->getSharedPtr(), pm_->getSharedPtr());
    
    auto [solution_found, trajectory] = planner.solve(1.0, false);
    
    if (!solution_found || trajectory.empty() || collision_checker_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "No valid trajectory found");
        return {false, {}};
    }
    
    // Validate trajectory
    for (const auto& state : trajectory) {
        if (!validateState(state)) {
            RCLCPP_WARN(this->get_logger(), "Invalid state in trajectory");
            return {false, {}};
        }
    }
    
    return {true, trajectory};
}

void BowPlannerInterface::publishTrajectory(const std::vector<bow::State>& trajectory)
{
    if (trajectory.empty()) return;
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05; // Thinner line
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f; // Slightly transparent
    
    marker.points.reserve(trajectory.size());
    for (const auto& state : trajectory) {
        geometry_msgs::msg::Point point;
        point.x = state(0, 0);
        point.y = state(1, 0);
        point.z = 0.02; // Slightly above ground
        marker.points.push_back(point);
    }
    
    trajectory_pub_->publish(marker);
}

void BowPlannerInterface::publishPose(const bow::State& state)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "robot_pose";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3; // Arrow length
    marker.scale.y = 0.05; // Arrow width
    marker.scale.z = 0.05; // Arrow height
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    
    marker.pose.position.x = state(0, 0);
    marker.pose.position.y = state(1, 0);
    marker.pose.position.z = 0.05;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, state(2, 0));
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    pose_pub_->publish(marker);
}

void BowPlannerInterface::publishObstacles(const std::vector<geometry_msgs::msg::Point>& points)
{
    if (points.empty()) return;
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "obstacles";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    
    marker.points = points;
    obstacles_pub_->publish(marker);
}

void BowPlannerInterface::publishCmdVel(double linear_vel, double angular_vel)
{
    // Clamp velocities to safe limits
    linear_vel = std::clamp(linear_vel, params_.min_speed, params_.max_speed);
    angular_vel = std::clamp(angular_vel, -params_.max_yawrate, params_.max_yawrate);
    
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    
    cmd_vel_pub_->publish(cmd_vel);
}

bool BowPlannerInterface::isGoalReached() 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    double dx = current_state_(0, 0) - goal_state_(0, 0);
    double dy = current_state_(1, 0) - goal_state_(1, 0);
    double distance = std::sqrt(dx * dx + dy * dy);
    
    return distance < params_.goal_radius;
}

void BowPlannerInterface::stopRobot()
{
    publishCmdVel(0.0, 0.0);
    
    // Reset velocities in state
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_(3, 0) = 0.0;
        current_state_(4, 0) = 0.0;
    }
    
    use_cached_trajectory_ = false;
    last_trajectory_.clear();
}

double BowPlannerInterface::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool BowPlannerInterface::validateState(const bow::State& state) const
{
    // Check for NaN or infinite values
    for (int i = 0; i < state.rows(); ++i) {
        for (int j = 0; j < state.cols(); ++j) {
            if (!std::isfinite(state(i, j))) {
                return false;
            }
        }
    }
    
    // Check velocity limits if velocities are present
    if (state.rows() > 3) {
        double linear_vel = state(3, 0);
        double angular_vel = state.rows() > 4 ? state(4, 0) : 0.0;
        
        if (std::abs(linear_vel) > params_.max_speed * 1.5 || 
            std::abs(angular_vel) > params_.max_yawrate * 1.5) {
            return false;
        }
    }
    
    return true;
}

std::pair<double, double> BowPlannerInterface::extractControlCommands(const std::vector<bow::State>& trajectory)
{
    if (trajectory.empty()) {
        return {0.0, 0.0};
    }
    
    // Use state at preferred speed index or the last available state
    int index = std::min(static_cast<int>(trajectory.size()) - 1, params_.pref_speed_index);
    const auto& target_state = trajectory[index];
    
    double linear_vel = (target_state.rows() > 3) ? target_state(3, 0) : 0.0;
    double angular_vel = (target_state.rows() > 4) ? target_state(4, 0) : 0.0;
    
    return {linear_vel, angular_vel};
}

void BowPlannerInterface::logStatus() const
{
    // std::lock_guard<std::mutex> lock(state_mutex_);
    
    // double dx = current_state_(0, 0) - goal_state_(0, 0);
    // double dy = current_state_(1, 0) - goal_state_(1, 0);
    // double distance = std::sqrt(dx * dx + dy * dy);
    
    // RCLCPP_INFO(this->get_logger(), 
    //            "Status - Position: (%.2f, %.2f), Goal: (%.2f, %.2f), Distance: %.2f, "
    //            "Linear vel: %.2f, Angular vel: %.2f, Loop: %zu",
    //            current_state_(0, 0), current_state_(1, 0),
    //            goal_state_(0, 0), goal_state_(1, 0),
    //            distance,
    //            current_state_(3, 0), current_state_(4, 0),
    //            loop_count_);
}

