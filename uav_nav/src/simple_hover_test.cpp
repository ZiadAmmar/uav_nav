#include <ros/ros.h>
#include "uav_nav/PIDController.hpp"
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>

class SimpleHoverTest {
private:
    ros::NodeHandle nh_;
    PIDController pid_controller_;
    
    ros::Publisher local_pos_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped hover_setpoint_;
    
public:
    SimpleHoverTest() {
        // Initialize ROS communications
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
        state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &SimpleHoverTest::stateCallback, this);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &SimpleHoverTest::poseCallback, this);
            
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
            
        // Set hover setpoint
        hover_setpoint_.pose.position.x = 0.0;
        hover_setpoint_.pose.position.y = 0.0;
        hover_setpoint_.pose.position.z = 1.5;
        hover_setpoint_.pose.orientation.w = 1.0;
    }
    
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        
        // Print current position for debugging
        ROS_INFO_THROTTLE(1.0, "Current Position - X: %.2f, Y: %.2f, Z: %.2f",
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
    }
    
    void run() {
        ros::Rate rate(20.0);
        
        // Wait for FCU connection
        while(ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }
        
        // Send a few setpoints before starting
        for(int i = 0; ros::ok() && i < 100; i++) {
            local_pos_pub_.publish(hover_setpoint_);
            ros::spinOnce();
            rate.sleep();
        }
        
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        ros::Time last_request = ros::Time::now();
        
        while(ros::ok()) {
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
            
            // Compute PID control command
            geometry_msgs::PoseStamped setpoint = pid_controller_.computeControlCommand(
                current_pose_,
                hover_setpoint_);
                
            // Publish setpoint
            local_pos_pub_.publish(setpoint);
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_hover_test");
    SimpleHoverTest hover_test;
    hover_test.run();
    return 0;
}
