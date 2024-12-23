#include "uav_nav/PIDController.hpp"

PIDController::PIDController()
    : x_integral_(0.0), y_integral_(0.0), z_integral_(0.0),
      x_prev_error_(0.0), y_prev_error_(0.0), z_prev_error_(0.0),
      x_derivative_filtered_(0.0), y_derivative_filtered_(0.0), z_derivative_filtered_(0.0)
{    
    loadParameters();
    last_time_ = ros::Time::now();
}

void PIDController::loadParameters() {
    // Load PID gains
    getAxisGains("x", x_gains_);
    getAxisGains("y", y_gains_);
    getAxisGains("z", z_gains_);

    // Load control parameters with default values
    nh_.param<bool>("pid_control/debug", debug_, false);
    nh_.param<double>("pid_control/deadband", deadband_, 0.05);
    
    // Error limits
    nh_.param<double>("pid_control/max_error/x", max_x_error_, 1.0);
    nh_.param<double>("pid_control/max_error/y", max_y_error_, 1.0);
    nh_.param<double>("pid_control/max_error/z", max_z_error_, 0.7);
    
    // Integral limits
    nh_.param<double>("pid_control/max_integral/x", max_x_integral_, 1.0);
    nh_.param<double>("pid_control/max_integral/y", max_y_integral_, 1.0);
    nh_.param<double>("pid_control/max_integral/z", max_z_integral_, 0.1);
    
    // Integral threshold
    nh_.param<double>("pid_control/integral_threshold", integral_threshold_, 0.2);
    
    // Derivative limits
    nh_.param<double>("pid_control/max_derivative/x", max_x_derivative_, 4.0);
    nh_.param<double>("pid_control/max_derivative/y", max_y_derivative_, 4.0);
    nh_.param<double>("pid_control/max_derivative/z", max_z_derivative_, 1.0);
    
    // Derivative filter coefficient
    nh_.param<double>("pid_control/derivative_filter_alpha", derivative_filter_alpha_, 0.7);
}

bool PIDController::getAxisGains(const std::string& axis, PIDGains& gains) {
    bool success = true;
    std::string prefix = "pid_control/" + axis + "/";
    
    success &= nh_.getParam(prefix + "kp", gains.kp);
    success &= nh_.getParam(prefix + "ki", gains.ki);
    success &= nh_.getParam(prefix + "kd", gains.kd);
    
    if (!success) {
        ROS_WARN("Failed to load PID gains for %s axis, using defaults", axis.c_str());
        // Default values as fallback
        gains = (axis == "z") ? PIDGains{10.7f, 0.02025f, 5.13f} :
               (axis == "y") ? PIDGains{12.0f, 0.027f, 6.0f} :
                              PIDGains{11.0f, 0.025f, 3.0f};
    }
    
    return success;
}

void PIDController::setGains(float kp, float ki, float kd, char axis) {
    PIDGains* gains;
    switch(axis) {
        case 'x': gains = &x_gains_; break;
        case 'y': gains = &y_gains_; break;
        case 'z': gains = &z_gains_; break;
        default: return;
    }
    gains->kp = kp;
    gains->ki = ki;
    gains->kd = kd;
}

geometry_msgs::PoseStamped PIDController::computeControlCommand(
    const geometry_msgs::PoseStamped& current_pose,
    const geometry_msgs::PoseStamped& target_pose) {
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    
    // Ensure dt is reasonable
    if (dt > 0.1) dt = 0.1;
    if (dt < 0.001) dt = 0.001;
    
    // Compute errors
    double x_error = target_pose.pose.position.x - current_pose.pose.position.x;
    double y_error = target_pose.pose.position.y - current_pose.pose.position.y;
    double z_error = target_pose.pose.position.z - current_pose.pose.position.z;
    
    // Print errors for debugging and tunning
    if(debug_){
        ROS_INFO_THROTTLE(1.0, "Position errors - X: %.3f, Y: %.3f, Z: %.3f", 
                        x_error, y_error, z_error);
        ROS_INFO_THROTTLE(1.0, "Position errors X: %.4f", x_error);
        ROS_INFO_THROTTLE(1.0, "Position errors Y: %.4f", y_error);
        ROS_INFO_THROTTLE(1.0, "Position errors Z: %.4f", z_error);
    }
    // Reset integral when in deadband
    if (std::abs(x_error) < deadband_) {
        x_error = 0.0;
        x_integral_ = 0.0;  
    }
    if (std::abs(y_error) < deadband_) {
        y_error = 0.0;
        y_integral_ = 0.0;  
    }    
    if (std::abs(z_error) < deadband_) {
        z_error = 0.0;
        z_integral_ = 0.0;  
    }

    // Error limits
    x_error = std::max(std::min(x_error, max_x_error_), -max_x_error_);
    y_error = std::max(std::min(y_error, max_y_error_), -max_y_error_);
    z_error = std::max(std::min(z_error, max_z_error_), -max_z_error_);
    
    // Integral calculations with anti-windup
    // Only accumulate integral when close to target
     if (std::abs(x_error) <= integral_threshold_) {
        x_integral_ = std::max(std::min(x_integral_ + x_error * dt, max_x_integral_), -max_x_integral_);
    } else {
        x_integral_ = 0.0;
    }

    if (std::abs(y_error) <= integral_threshold_) {
        y_integral_ = std::max(std::min(y_integral_ + y_error * dt, max_y_integral_), -max_y_integral_);
    } else {
        y_integral_ = 0.0;
    }

    if (std::abs(z_error) <= integral_threshold_) {
        z_integral_ = std::max(std::min(z_integral_ + z_error * dt, max_z_integral_), -max_z_integral_);
    } else {
        z_integral_ = 0.0;
    }

    // Filtered derivatives 
    double x_derivative = (x_error - x_prev_error_) / dt;
    double y_derivative = (y_error - y_prev_error_) / dt;
    double z_derivative = (z_error - z_prev_error_) / dt;
    
    x_derivative_filtered_ = derivative_filter_alpha_ * x_derivative_filtered_ + (1 - derivative_filter_alpha_) * x_derivative;
    y_derivative_filtered_ = derivative_filter_alpha_ * y_derivative_filtered_ + (1 - derivative_filter_alpha_) * y_derivative;
    z_derivative_filtered_ = derivative_filter_alpha_ * z_derivative_filtered_ + (1 - derivative_filter_alpha_) * z_derivative;
    
    x_derivative = x_derivative_filtered_;
    y_derivative = y_derivative_filtered_;
    z_derivative = z_derivative_filtered_;

    // Limit derivatives
    x_derivative = std::max(std::min(x_derivative, max_x_derivative_), -max_x_derivative_);
    y_derivative = std::max(std::min(y_derivative, max_y_derivative_), -max_y_derivative_);
    z_derivative = std::max(std::min(z_derivative, max_z_derivative_), -max_z_derivative_);

    // Store errors for next iteration
    x_prev_error_ = x_error;
    y_prev_error_ = y_error;
    z_prev_error_ = z_error;
    
    // Calculate control outputs
    double x_output = x_gains_.kp * x_error + 
                     x_gains_.ki * x_integral_ +
                     x_gains_.kd * x_derivative;
                     
    double y_output = y_gains_.kp * y_error +
                     y_gains_.ki * y_integral_ +
                     y_gains_.kd * y_derivative;
                     
    double z_output = z_gains_.kp * z_error +
                     z_gains_.ki * z_integral_ +
                     z_gains_.kd * z_derivative;

    // Print control outputs for debugging and tuning
    if(debug_){
        ROS_INFO_THROTTLE(1.0, "Control outputs - X: %.3f, Y: %.3f, Z: %.3f",
                        x_output, y_output, z_output);
        ROS_INFO_THROTTLE(1.0, "Control outputs X: %.3f", x_output);
        ROS_INFO_THROTTLE(1.0, "Control outputs Y: %.3f", y_output);
        ROS_INFO_THROTTLE(1.0, "Control outputs Z: %.3f", z_output);
    }
    // Set position commands
    geometry_msgs::PoseStamped control_command;
    control_command.header.stamp    = current_time;
    control_command.header.frame_id = "map";
    control_command.pose.position.x = current_pose.pose.position.x + x_output * dt;
    control_command.pose.position.y = current_pose.pose.position.y + y_output * dt;
    control_command.pose.position.z = current_pose.pose.position.z + z_output * dt;

    // Maintain current orientation
    control_command.pose.orientation = current_pose.pose.orientation;
    
    last_time_ = current_time;
    return control_command;
}

void PIDController::reset() {
    x_integral_   = y_integral_   = z_integral_   = 0.0;
    x_prev_error_ = y_prev_error_ = z_prev_error_ = 0.0;
    x_derivative_filtered_ = y_derivative_filtered_ = z_derivative_filtered_ = 0.0;
    last_time_ = ros::Time::now();
}