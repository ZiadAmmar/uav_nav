#include <ros/ros.h>
#include <signal.h>
#include "uav_nav/FlightController.hpp"

// Global pointer for clean shutdown
FlightController* controller_ptr = nullptr;

// Signal handler for clean shutdown
void signalHandler(int signum) {
    //TODO: I can initiate safe landing procedure here 
    ROS_INFO("Shutdown signal received!");
    ros::shutdown();
}

bool checkRequiredParameters(ros::NodeHandle& nh) {
    // Check for required parameters
    std::vector<std::string> required_params = {
        "/mission/takeoff_altitude",
        "/mission/waypoint_tolerance",
        "/mission/holding_time",
    };

    bool all_params_found = true;
    for (const auto& param : required_params) {
        if (!nh.hasParam(param)) {
            ROS_ERROR("Required parameter '%s' not found", param.c_str());
            all_params_found = false;
        }
    }
    return all_params_found;
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "uav_navigation_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // Register signal handler for clean shutdown
    signal(SIGINT, signalHandler);

    try {
        ROS_INFO("Starting UAV Navigation Node...");

        // Check if all required parameters are loaded
        if (!checkRequiredParameters(nh)) 
            ROS_WARN("Missing required parameters. Please check your parameter file. Using default values.");
        

        // Create and initialize flight controller
        FlightController controller;
        controller_ptr = &controller;

        ROS_INFO("Initializing UAV Navigation System...");
        controller.init();

        // Wait a moment to ensure proper initialization
        ros::Duration(1.0).sleep();

        // Print mission summary
        ROS_INFO("Mission Parameters:");
        float takeoff_altitude;
        nh.getParam("/mission/takeoff_altitude", takeoff_altitude);
        ROS_INFO("- Takeoff Altitude: %.2f meters", takeoff_altitude);
        ROS_INFO("Starting mission execution...");

        // Run the controller
        controller.run();

        // Clean shutdown
        controller_ptr = nullptr;
        ROS_INFO("UAV Navigation Node completed successfully");

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in UAV Navigation Node: %s", e.what());
        if (controller_ptr != nullptr) {
            controller_ptr = nullptr;
        }
        return 1;
    }

    return 0;
}