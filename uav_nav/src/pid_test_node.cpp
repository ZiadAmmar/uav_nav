#include <ros/ros.h>
#include "uav_nav/PIDController.hpp"
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>

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
    ros::Time position_start_time_;
    double position_hold_time_;

    //Flags
    bool initial_takeoff_done_;
    
public:
    PIDTester() : current_position_index_(0), initial_takeoff_done_(false) {
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
        
        // Set initial PID gains (For tuning then modify the default values after)
        // pid_controller_.setGains(11.0f, 0.025f, 3.0f, 'x');  
        // pid_controller_.setGains(12.0f, 0.027f, 6.0f, 'y');
        // pid_controller_.setGains(10.7f, 0.02025f, 5.13f, 'z');
        
        position_hold_time_ = 20.0; // Hold each position for 20 seconds
        position_start_time_ = ros::Time::now();
    }
    
    void setupTestPositions() {
        geometry_msgs::PoseStamped pos;
        pos.pose.orientation.w = 1.0;
        float altitude = 1.5;
        // Takeoff position
        pos.pose.position.x = current_pose_.pose.position.x;
        pos.pose.position.y = current_pose_.pose.position.y;
        pos.pose.position.z = altitude;  // 1.5 meters height
        test_positions_.push_back(pos);
        
        // Chandelier_01_004
        pos.pose.position.x = 1.221061;
        pos.pose.position.x = -1.592040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);
        
        // Chandelier_01_003
        pos.pose.position.x = 7.824061;
        pos.pose.position.x = -1.59;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);

        // Chandelier_01_004
        pos.pose.position.x = 1.221061;
        pos.pose.position.x = -1.592040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);
        
        // Chandelier_01_001
        pos.pose.position.x = 1.221061;
        pos.pose.position.x = 4.155740 ;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);

        // Chandelier_01_004
        pos.pose.position.x = 1.221061;
        pos.pose.position.x = -1.592040;
        pos.pose.position.z = altitude;
        test_positions_.push_back(pos);

        // Chandelier_01_002
        pos.pose.position.x = -6.451061;
        pos.pose.position.x = -0.692040;
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
        // Print current position for debugging and tunning
        ROS_INFO_THROTTLE(1.0, "Current Position - X: %.2f, Y: %.2f, Z: %.2f",
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
        // ROS_INFO_THROTTLE(1.0, "Current Position X: %.3f", current_pose_.pose.position.x);
        // ROS_INFO_THROTTLE(1.0, "Current Position Y: %.3f", current_pose_.pose.position.y);
        // ROS_INFO_THROTTLE(1.0, "Current Position Z: %.3f", current_pose_.pose.position.z);
        ROS_INFO_THROTTLE(1.0, "-----------------");

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
            // Try to switch to offboard mode
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
            
            // Check if it's time to move to next position
            if(ros::Time::now() - position_start_time_ > ros::Duration(position_hold_time_)) {
                if (!initial_takeoff_done_) {
                        // After the first hold, set flag and move to the next position
                        initial_takeoff_done_   = true;
                        current_position_index_ = 0;
                    } else {
                        // Normal position increment
                        current_position_index_ = (current_position_index_ + 1) % test_positions_.size();
                    }                
                position_start_time_ = ros::Time::now();
                ROS_INFO("-----------------------------------------------");
                ROS_INFO("Moving to position %d", current_position_index_);
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
