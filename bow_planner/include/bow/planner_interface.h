#ifndef BOW_PLANNER_INTERFACE_HPP
#define BOW_PLANNER_INTERFACE_HPP

#include <memory>
#include <vector>
#include <atomic>
#include <mutex>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "bow/BOW.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Parameters structure for BOW planner
 */
struct BowPlannerParams {
    double dt = 0.2;
    double predict_time = 3.0;
    double robot_radius = 0.25;
    double goal_radius = 0.6;
    int pref_speed_index = 5;
    int num_samples = 15;
    double max_speed = 1.0;
    double min_speed = 0.0;
    double max_yawrate = 0.6981;
    double map_resolution = 0.069;
    
    // Additional optimization parameters
    double position_tolerance = 1e-3;
    double angle_tolerance = 1e-2;
    int max_planning_iterations = 100;

    std::vector<double> boundary; // x_min, x_max, y_min, y_max
};

/**
 * @brief Optimized BOW Planner Interface for ROS2
 * 
 * This class implements an optimized version of the BOW (Bezier Optimal Waypoint) 
 * planner interface with improved performance, error handling, and thread safety.
 */
class BowPlannerInterface : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    BowPlannerInterface();
    
    /**
     * @brief Destructor
     */
    ~BowPlannerInterface() = default;

private:
    // Configuration and parameters
    BowPlannerParams params_;
    std::shared_ptr<param_manager> pm_;
    
    // State variables with thread safety
    bow::State current_state_;
    bow::Point goal_state_;
    std::mutex state_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> goal_reached_{false};
    std::atomic<size_t> loop_count_{0};
    
    // BOW planner components
    bow::CCPtr collision_checker_;
    std::mutex collision_checker_mutex_;
    
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vicon_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Cached data for performance
    std::vector<bow::State> last_trajectory_;
    rclcpp::Time last_pointcloud_time_;
    bool use_cached_trajectory_ = false;
    
    /**
     * @brief Initialize ROS2 parameters
     */
    void initializeParameters();
    
    /**
     * @brief Setup ROS2 publishers and subscribers
     */
    void setupPublishersAndSubscribers();
    
    /**
     * @brief Setup control timer
     */
    void setupTimer();
    
    /**
     * @brief Callback for Vicon odometry data
     * @param msg Odometry message
     */
    void viconCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Callback for point cloud data (obstacles)
     * @param msg PointCloud2 message
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /**
     * @brief Callback for goal pose from RViz
     * @param msg PoseStamped message
     */
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief Main control loop timer callback
     */
    void controlTimerCallback();
    
    /**
     * @brief Plan trajectory using BOW planner
     * @return Pair of solution status and trajectory
     */
    std::pair<bool, std::vector<bow::State>> planTrajectory();
    
    /**
     * @brief Publish trajectory visualization
     * @param trajectory Vector of states representing the trajectory
     */
    void publishTrajectory(const std::vector<bow::State>& trajectory);
    
    /**
     * @brief Publish robot pose visualization
     * @param state Current robot state
     */
    void publishPose(const bow::State& state);
    
    /**
     * @brief Publish obstacles visualization
     * @param points Obstacle points
     */
    void publishObstacles(const std::vector<geometry_msgs::msg::Point>& points);
    
    /**
     * @brief Publish velocity commands
     * @param linear_vel Linear velocity
     * @param angular_vel Angular velocity
     */
    void publishCmdVel(double linear_vel, double angular_vel);
    
    /**
     * @brief Check if goal is reached
     * @return True if goal is reached
     */
    bool isGoalReached();
    
    /**
     * @brief Stop the robot
     */
    void stopRobot();
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Input angle in radians
     * @return Normalized angle
     */
    static double normalizeAngle(double angle);
    
    /**
     * @brief Validate state for safety
     * @param state State to validate
     * @return True if state is valid
     */
    bool validateState(const bow::State& state) const;
    
    /**
     * @brief Extract control commands from trajectory
     * @param trajectory Planned trajectory
     * @return Pair of linear and angular velocities
     */
    std::pair<double, double> extractControlCommands(const std::vector<bow::State>& trajectory);
    
    /**
     * @brief Log system status
     */
    void logStatus() const;
};

#endif // BOW_PLANNER_INTERFACE_HPP