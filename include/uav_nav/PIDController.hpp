#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>

class PIDController {
private:
    // PID gains
    struct PIDGains {
        float kp, ki, kd;
    };
    
    PIDGains x_gains_, y_gains_, z_gains_;
    
    // Error integrals
    double x_integral_, y_integral_, z_integral_;
    
    // Previous errors for derivative
    double x_prev_error_, y_prev_error_, z_prev_error_;
    
    // Filtered derivative for Z axis
    double x_derivative_filtered_, y_derivative_filtered_, z_derivative_filtered_;
    
    // Time management
    ros::Time last_time_;
    
    // Control parameters
    double deadband_;
    double max_x_error_, max_y_error_, max_z_error_;
    double max_x_integral_, max_y_integral_, max_z_integral_;
    double integral_threshold_;
    double max_x_derivative_, max_y_derivative_, max_z_derivative_;
    double derivative_filter_alpha_;
    bool debug_;

    // ROS NodeHandle for parameter server access
    ros::NodeHandle nh_;

    // Helper function to load parameters
    void loadParameters();
    bool getAxisGains(const std::string& axis, PIDGains& gains);

public:
    PIDController();
    
    void setGains(float kp, float ki, float kd, char axis);
    
    geometry_msgs::PoseStamped computeControlCommand(
        const geometry_msgs::PoseStamped& current_pose,
        const geometry_msgs::PoseStamped& target_pose);
        
    void reset();
};