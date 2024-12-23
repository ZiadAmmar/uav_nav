#include <ros/ros.h>
#include "uav_nav/PIDController.hpp"
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class PIDTester {
private:
    ros::NodeHandle nh_;
    PIDController pid_controller_;
    
    // Publishers & Subscribers
    ros::Publisher local_pos_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    
    // Service clients
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    
    // Current state
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    
    // Test parameters
    std::vector<geometry_msgs::PoseStamped> test_positions_;
    int current_position_index_;
    
    // Position tracking parameters
    const float POSITION_THRESHOLD = 0.1;  // meters
    const float STABLE_DURATION = 2.0;     // seconds
    ros::Time position_reached_time_;
    bool position_reached_;
    bool initial_takeoff_done_;
    
public:
    PIDTester() : current_position_index_(0), position_reached_(false), initial_takeoff_done_(false) {
        // Initialize ROS communications
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
        state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &PIDTester::stateCallback, this);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &PIDTester::poseCallback, this);
            
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
            
        // Set up test positions
        setupTestPositions();
    }
    
    // Check if current position matches target position within threshold
    bool hasReachedPosition(const geometry_msgs::PoseStamped& target) {
        double dx = current_pose_.pose.position.x - target.pose.position.x;
        double dy = current_pose_.pose.position.y - target.pose.position.y;
        double dz = current_pose_.pose.position.z - target.pose.position.z;
        
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance < POSITION_THRESHOLD) {
            if (!position_reached_) {
                position_reached_ = true;
                position_reached_time_ = ros::Time::now();
            }
            // Check if position has been maintained for STABLE_DURATION
            return (ros::Time::now() - position_reached_time_).toSec() >= STABLE_DURATION;
        }
        
        position_reached_ = false;
        return false;
    }
    
    void setupTestPositions() {
        geometry_msgs::PoseStamped pos;
        pos.pose.orientation.w = 1.0;
        float altitude = 1.3;
        // // Takeoff position
        pos.pose.position.x = current_pose_.pose.position.x;
        pos.pose.position.y = current_pose_.pose.position.y;
        pos.pose.position.z = altitude;  
        test_positions_.push_back(pos);
        
        // Chandelier_01_004
        pos.pose.position.x = 1.221061;
        pos.pose.position.y = -1.592040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);
        
        // Chandelier_01_003
        pos.pose.position.x = 7.824061;
        pos.pose.position.y = -1.59;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);

        // Chandelier_01_004
        pos.pose.position.x = 1.221061;
        pos.pose.position.y = -1.592040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);
        
        // Chandelier_01_001
        pos.pose.position.x = 1.221061;
        pos.pose.position.y = 4.155740 ;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);

        // Chandelier_01_004
        pos.pose.position.x = 1.221061;
        pos.pose.position.y = -1.592040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);

        // Chandelier_01_002
        pos.pose.position.x = -6.451061;
        pos.pose.position.y = -0.692040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);
        
        // Return to home
        pos.pose.position.x = 0.0;
        pos.pose.position.y = 0.0;
        test_positions_.push_back(pos);
    }
    
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        
        // Print current position and target position for debugging
        ROS_INFO_THROTTLE(1.0, "Current Position  - X: %.3f, Y: %.3f, Z: %.3f",
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
            
        ROS_INFO_THROTTLE(1.0, "Target Position   - X: %.3f, Y: %.3f, Z: %.3f",
            test_positions_[current_position_index_].pose.position.x,
            test_positions_[current_position_index_].pose.position.y,
            test_positions_[current_position_index_].pose.position.z);
    }
    
    void run() {
        ros::Rate rate(20.0);
        
        // Wait for FCU connection
        while(ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
        
        // Send a few setpoints before starting
        geometry_msgs::PoseStamped init_pose = test_positions_[0];
        for(int i = 0; ros::ok() && i < 100; i++) {
            local_pos_pub_.publish(init_pose);
            ros::spinOnce();
            rate.sleep();
        }
        
        // Switch to offboard mode and arm
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        ros::Time last_request = ros::Time::now();
        
        while(ros::ok()) {
            // Try to switch to offboard mode and arm
            if(current_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if(set_mode_client_.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else if(!current_state_.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if(arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            
            // Check if current position is reached
            if(hasReachedPosition(test_positions_[current_position_index_])) {
                if (!initial_takeoff_done_) {
                    // After successful takeoff, move to first waypoint
                    initial_takeoff_done_ = true;
                    current_position_index_ = 1;  // Start with first waypoint after takeoff
                    ROS_INFO("Takeoff complete, moving to first waypoint");
                } else {
                    // Move to next position
                    current_position_index_ = (current_position_index_ + 1) % test_positions_.size();
                    ROS_INFO("Position reached, moving to position %d", current_position_index_);
                }
                position_reached_ = false;  // Reset for next position
            }
            
            // Compute PID control command
            geometry_msgs::PoseStamped setpoint = pid_controller_.computeControlCommand(
                current_pose_,
                test_positions_[current_position_index_]);
                
            // Publish setpoint
            local_pos_pub_.publish(setpoint);
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_test_node");
    PIDTester tester;
    tester.run();
    return 0;
}