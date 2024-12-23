#pragma once
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h> 
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Path.h>
#include "PIDController.hpp"

class FlightController {
private:
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher local_pos_pub_;
    ros::Publisher attitude_target_pub_;
    ros::Publisher waypoint_pub_;
    
    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber fast_planner_pos_cmd_sub_; // Fast Planner trajectory subscriber
    
    // Service clients
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    // Current state
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    quadrotor_msgs::PositionCommand current_trajectory_point_; // Current point from fast_planner
    bool trajectory_received_; // Flag to track if received trajectory points
    int current_waypoint_index_;

    // Controllers
    PIDController position_controller_;
    
    // Mission state
    enum class MissionState {
        IDLE,
        ARMING,
        TAKEOFF,
        WAITING_FOR_PLANNER, // New state
        TRAJECTORY_FOLLOWING,
        HOLDING,
        LANDING
    } mission_state_;
    
    // Mission parameters
    double takeoff_altitude_;
    double waypoint_tolerance_;
    double holding_time_;
    ros::Time state_entry_time_;
    ros::Time last_trajectory_time_;
    bool PIDControl_enabled_;

    // Waypoints
    std::vector<geometry_msgs::Point> waypoints_;
    
    // Helper functions to initialize parameters and waypoints
    void setupParameters();
    void initializeWaypoints();

    // Callback functions
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void trajectoryCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    
    // Mode setting functions
    bool setOffboardMode();
    bool setHoldMode();
    bool setLandMode();
    bool armVehicle();
    
    // State execution functions
    void executeArming();
    void executeTakeoff();
    void executeWaitForPlanner();
    void executeTrajectory();
    void executeHolding();
    void executeLanding();
    
    // Helper functions
    bool reachedPosition(const geometry_msgs::Point& target, double tolerance);
    void publishSetpoint(const geometry_msgs::PoseStamped& setpoint);
    void publishWaypoints();
    bool isTrajectoryComplete();

public:
    FlightController();
    void init();
    void run();
};