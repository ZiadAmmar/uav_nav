#include "uav_nav/FlightController.hpp"

FlightController::FlightController()
    : mission_state_(MissionState::IDLE),
      takeoff_altitude_(1.5),
      waypoint_tolerance_(0.1),
      holding_time_(15.0),
      trajectory_received_(false),
      current_waypoint_index_(0),
      PIDControl_enabled_(true)
{
    // Initialize ROS communications
    // Publishers - TOFIX
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    attitude_target_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>
        ("mavros/setpoint_raw/attitude", 10);
    waypoint_pub_ = nh_.advertise<nav_msgs::Path>
        ("/waypoint_generator/waypoints", 1);

    // Subscribers
    state_sub_ = nh_.subscribe<mavros_msgs::State>
        ("mavros/state", 10, &FlightController::stateCallback, this);
    local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, &FlightController::poseCallback, this);
    fast_planner_pos_cmd_sub_ = nh_.subscribe<quadrotor_msgs::PositionCommand>
        ("/fast_planner/position_cmd", 10, &FlightController::trajectoryCallback, this);

    // Service Clients
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
}

void FlightController::init() {
    setupParameters();
    initializeWaypoints();
    state_entry_time_ = ros::Time::now();
    ROS_INFO("Flight Controller initialized successfully");
}

void FlightController::setupParameters() {
    // Load parameters from ROS parameter server
    if (!nh_.getParam("/mission/takeoff_altitude", takeoff_altitude_)) 
        ROS_WARN("Takeoff altitude not set, using default: %.2f m", takeoff_altitude_);
    
    if (!nh_.getParam("/mission/waypoint_tolerance", waypoint_tolerance_)) 
        ROS_WARN("Waypoint tolerance not set, using default: %.2f m", waypoint_tolerance_);
    
    if (!nh_.getParam("/mission/holding_time", holding_time_)) 
        ROS_WARN("Holding time not set, using default: %.2f s", holding_time_);
    
    if (!nh_.getParam("/pid_control/enabled", PIDControl_enabled_)) 
        ROS_WARN("PIDControl Enable flag not set, using default: %s", PIDControl_enabled_ ? "Enabled" : "Disabled");
    
    ROS_INFO("Mission parameters loaded : Takeoff altitude: %.2f m, Tolerance: %.2f m, Holding time: %.2f s",
             takeoff_altitude_, waypoint_tolerance_, holding_time_);
    if (PIDControl_enabled_) {
        ROS_INFO("Using PID Position Control");
    } else {
        ROS_INFO("PID Position Control Disabled, Using default controller");
    }
}
void FlightController::initializeWaypoints() {
    // Define waypoints in the map frame
    geometry_msgs::Point wp;
    
    // Starting point (after takeoff)
    wp.x = 1.221061; wp.y = -1.592040; wp.z = takeoff_altitude_; // Chandelier_01_004
    waypoints_.push_back(wp);
    
    wp.x = 7.824061; wp.y = -1.59; wp.z = takeoff_altitude_; // Chandelier_01_003
    waypoints_.push_back(wp);
    
    wp.x = 1.221061; wp.y = -1.592040; wp.z = takeoff_altitude_; // Chandelier_01_004
    waypoints_.push_back(wp);
    
    wp.x = 1.221061; wp.y = 4.155740; wp.z = takeoff_altitude_; // Chandelier_01_001
    waypoints_.push_back(wp);
    
    wp.x = 1.221061; wp.y = -1.592040; wp.z = takeoff_altitude_; // Chandelier_01_004
    waypoints_.push_back(wp);
    
    wp.x = -6.451061; wp.y = -0.692040; wp.z = takeoff_altitude_; // Chandelier_01_002
    waypoints_.push_back(wp);

    // Return to starting point
    wp.x = 0.0; wp.y = 0.0; wp.z = takeoff_altitude_;
    waypoints_.push_back(wp);
    
    ROS_INFO("Initialized %lu waypoints for the mission", waypoints_.size());
}

// Callback Functions
void FlightController::trajectoryCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    current_trajectory_point_ = *msg;
    trajectory_received_ = true;
    last_trajectory_time_ = ros::Time::now();
    
    // Check if reached current waypoint
    if (current_waypoint_index_ < waypoints_.size() && 
        reachedPosition(waypoints_[current_waypoint_index_], waypoint_tolerance_)) {
        current_waypoint_index_++;
        if (current_waypoint_index_ < waypoints_.size()) {
            // Publish next waypoint
            publishWaypoints();
        } else {
            // All waypoints completed
            ROS_INFO("All waypoints reached");
            mission_state_ = MissionState::HOLDING;
            state_entry_time_ = ros::Time::now();
        }
    }
}

void FlightController::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
    
    // Monitor connection status
    static bool was_connected = false;
    if (current_state_.connected != was_connected) {
        if (current_state_.connected) {
            ROS_INFO("Connected to FCU");
        } else {
            ROS_WARN("Lost connection to FCU");
        }
        was_connected = current_state_.connected;
    }
}

void FlightController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}
// Mode Setting functions
bool FlightController::setOffboardMode() {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
        return true;
    }
    ROS_WARN("Failed to set OFFBOARD mode");
    return false;
}

bool FlightController::setHoldMode() {
    mavros_msgs::SetMode hold_set_mode;
    hold_set_mode.request.custom_mode = "AUTO.LOITER";
    
    if (set_mode_client_.call(hold_set_mode) && hold_set_mode.response.mode_sent) {
        ROS_INFO("Hold mode enabled");
        return true;
    }
    ROS_WARN("Failed to set HOLD mode");
    return false;
}

bool FlightController::setLandMode() {
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    
    if (set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent) {
        ROS_INFO("Land mode enabled");
        return true;
    }
    ROS_WARN("Failed to set LAND mode");
    return false;
}

bool FlightController::armVehicle() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

// State Execution Functions
void FlightController::executeArming() {
    //pre-flight check
    if (!current_state_.connected) {
        ROS_ERROR("No FCU connection");
        return;
    }
    // Attempt to arm if not armed
    if (!current_state_.armed) {
        if (armVehicle()) {
            mission_state_ = MissionState::TAKEOFF;
            state_entry_time_ = ros::Time::now();
            ROS_INFO("Transitioning to TAKEOFF state");
        }
    }
}

void FlightController::executeWaitForPlanner() {
    static const ros::Duration PLANNER_TIMEOUT(15.0);
    static bool waypoints_published = false;

    // Initialize waypoint index
    current_waypoint_index_ = 0;
    
    // Maintain current altitude while waiting
    geometry_msgs::PoseStamped holding_setpoint = current_pose_;
    holding_setpoint.pose.position.z = takeoff_altitude_;
    publishSetpoint(holding_setpoint);
    
    // Publish first waypoint
    publishWaypoints();
    
    if (trajectory_received_) {
        mission_state_ = MissionState::TRAJECTORY_FOLLOWING;
        ROS_INFO("Trajectory received, starting execution");
        return;
    }
    
    // Check for timeout
    if (ros::Time::now() - state_entry_time_ > PLANNER_TIMEOUT) {
        ROS_ERROR("Timeout waiting for trajectory planner");
        mission_state_ = MissionState::LANDING;
    }
}


void FlightController::executeTrajectory() {
    static const ros::Duration TRAJECTORY_TIMEOUT(1.0); // 1 second timeout for trajectory updates

    // Check if still receiving trajectory updates
    if (ros::Time::now() - last_trajectory_time_ > TRAJECTORY_TIMEOUT) {
        ROS_WARN_THROTTLE(1.0, "No recent trajectory updates received");
        if (isTrajectoryComplete()) {
            if (setHoldMode()) {
                ROS_INFO("Trajectory complete, transitioning to HOLDING state");
                mission_state_ = MissionState::HOLDING;
                state_entry_time_ = ros::Time::now();
                return;
            }
        }
    }

    // Convert fast_planner position command to PoseStamped
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "map";
    target_pose.pose.position = current_trajectory_point_.position;
    
    // Use current orientation or compute desired orientation from velocity
    if (current_trajectory_point_.velocity.x != 0 || current_trajectory_point_.velocity.y != 0) {
        // Compute desired yaw from velocity vector
        double yaw = atan2(current_trajectory_point_.velocity.y, current_trajectory_point_.velocity.x);
        // Convert yaw to quaternion (assuming level flight)
        target_pose.pose.orientation.x = 0;
        target_pose.pose.orientation.y = 0;
        target_pose.pose.orientation.z = sin(yaw / 2);
        target_pose.pose.orientation.w = cos(yaw / 2);
    } else {
        // Maintain current orientation if no velocity
        target_pose.pose.orientation = current_pose_.pose.orientation;
    }

    // Use PID controller to compute control command
    publishSetpoint(target_pose);
}

void FlightController::executeHolding() {
    // Publish current position as setpoint
    publishSetpoint(current_pose_);
    
    // Check if held position long enough
    if (ros::Time::now() - state_entry_time_ > ros::Duration(holding_time_)) {
        mission_state_ = MissionState::LANDING;
        ROS_INFO("Hold time complete, transitioning to LANDING");
    }
}

void FlightController::executeLanding() {
    static const double GROUND_THRESHOLD = 0.1; // meters
    static const ros::Duration LANDING_TIMEOUT(30.0); // 30 seconds
    
    if (setLandMode()) {
        // Monitor landing progress
        if (current_pose_.pose.position.z < GROUND_THRESHOLD) {
            ROS_INFO("Landing complete");
            ros::shutdown();
        } else if (ros::Time::now() - state_entry_time_ > LANDING_TIMEOUT) {
            ROS_ERROR("Landing timeout reached");
            ros::shutdown();
        }
    }
}

// Helper Functions
bool FlightController::reachedPosition(const geometry_msgs::Point& target, double tolerance) {
    float dx = current_pose_.pose.position.x - target.x;
    float dy = current_pose_.pose.position.y - target.y;
    float dz = current_pose_.pose.position.z - target.z;
    
    return sqrt(dx*dx + dy*dy + dz*dz) < tolerance;
}

void FlightController::publishSetpoint(const geometry_msgs::PoseStamped& setpoint) {
    // Validate setpoint before publishing
    if (std::isnan(setpoint.pose.position.x) || 
        std::isnan(setpoint.pose.position.y) || 
        std::isnan(setpoint.pose.position.z)) {
        ROS_ERROR("Invalid setpoint detected (NaN values). Using current position.");
        local_pos_pub_.publish(current_pose_);
        return;
    }
    if(PIDControl_enabled_){
        // Compute control command using PID controller
        geometry_msgs::PoseStamped control_output = 
            position_controller_.computeControlCommand(current_pose_, setpoint);
        
        // Publish position setpoint
        local_pos_pub_.publish(control_output);
    } else {
        // Bypass PID controller and publish setpoint directly
        local_pos_pub_.publish(setpoint);
    }
}

void FlightController::publishWaypoints() {
    //TO FIX
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "map";
    
    // Get current waypoint
    if (current_waypoint_index_ < waypoints_.size()) {
        target_pose.pose.position = waypoints_[current_waypoint_index_];
        // Set orientation (assuming desired yaw along path)
        if (current_waypoint_index_ + 1 < waypoints_.size()) {
            // Calculate desired yaw based on direction to next waypoint
            double dx = waypoints_[current_waypoint_index_ + 1].x - waypoints_[current_waypoint_index_].x;
            double dy = waypoints_[current_waypoint_index_ + 1].y - waypoints_[current_waypoint_index_].y;
            double yaw = atan2(dy, dx);
            
            // Convert yaw to quaternion
            target_pose.pose.orientation.x = 0;
            target_pose.pose.orientation.y = 0;
            target_pose.pose.orientation.z = sin(yaw / 2);
            target_pose.pose.orientation.w = cos(yaw / 2);
        } else {
            // For last waypoint, maintain current orientation
            target_pose.pose.orientation = current_pose_.pose.orientation;
        }
        
        // Publish target pose for FastPlanner
        //TO FIX Publish waypoints for fast planner
        local_pos_pub_.publish(target_pose);
        
        ROS_INFO("Publishing target waypoint %d: [%.2f, %.2f, %.2f]", 
                 current_waypoint_index_,
                 target_pose.pose.position.x,
                 target_pose.pose.position.y,
                 target_pose.pose.position.z);
    }
}

bool FlightController::isTrajectoryComplete() {
    // Check if reached the final waypoint
    geometry_msgs::Point final_waypoint = waypoints_.back();
    
    // Check position tolerance
    return reachedPosition(final_waypoint, waypoint_tolerance_);
}

void FlightController::executeTakeoff() {
    static const ros::Duration timeout(30.0); // 30 second timeout for takeoff
    
    geometry_msgs::PoseStamped takeoff_pose = current_pose_;
    takeoff_pose.pose.position.z = takeoff_altitude_;
    
    publishSetpoint(takeoff_pose);
    
    // Check for timeout
    if (ros::Time::now() - state_entry_time_ > timeout) {
        ROS_ERROR("Takeoff timeout reached");
        mission_state_ = MissionState::LANDING;
        return;
    }
    
    // Check if reached takeoff altitude
    geometry_msgs::Point takeoff_point;
    takeoff_point.x = takeoff_pose.pose.position.x;
    takeoff_point.y = takeoff_pose.pose.position.y;
    takeoff_point.z = takeoff_pose.pose.position.z;
    if (reachedPosition(takeoff_point, waypoint_tolerance_)) {
        mission_state_ = MissionState::WAITING_FOR_PLANNER;
        state_entry_time_ = ros::Time::now();
        ROS_INFO("Takeoff complete, waiting for trajectory planner");
    }
}

void FlightController::run() {
    ros::Rate rate(20.0);
    
    // Wait for FCU connection
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Send a few setpoints before starting
    ROS_INFO("FCU connected, sending initial setpoints...");
    for (int i = 0; ros::ok() && i < 100; i++) {
        publishSetpoint(current_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Starting mission control loop");
    while (ros::ok()) {
        // Handle different mission states
        switch (mission_state_) {
            case MissionState::IDLE:
                if (setOffboardMode()) {
                    mission_state_ = MissionState::ARMING;
                    state_entry_time_ = ros::Time::now();
                }
                break;
                
            case MissionState::ARMING:
                executeArming();
                break;
                
            case MissionState::TAKEOFF:
                executeTakeoff();
                break;
                
            case MissionState::WAITING_FOR_PLANNER:
                executeWaitForPlanner();
                break;
                
            case MissionState::TRAJECTORY_FOLLOWING:
                executeTrajectory();
                break;
                
            case MissionState::HOLDING:
                executeHolding();
                break;
                
            case MissionState::LANDING:
                executeLanding();
                break;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}