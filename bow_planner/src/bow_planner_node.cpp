#include "bow/planner_interface.h"
#include <memory>
#include <csignal>
#include <atomic>

// Global flag for graceful shutdown
std::atomic<bool> shutdown_requested{false};

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutdown signal received (%d). Shutting down gracefully...", signum);
    shutdown_requested = true;
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    try {
        // Initialize ROS 2
        rclcpp::init(argc, argv);
        
        // Set up signal handler for graceful shutdown
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
        
        // Create node
        auto node = std::make_shared<BowPlannerInterface>();
        
        RCLCPP_INFO(node->get_logger(), "BOW Planner Interface started. Press Ctrl+C to exit.");
        
        // Spin until shutdown is requested
        while (rclcpp::ok() && !shutdown_requested) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        RCLCPP_INFO(node->get_logger(), "BOW Planner Interface shutting down...");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception in main");
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}