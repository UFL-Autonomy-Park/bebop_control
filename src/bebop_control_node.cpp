#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "bebop_control/DirtyDerivative.hpp"

using namespace std::chrono_literals;

class BebopControlNode : public rclcpp::Node {
public:
    BebopControlNode() : Node("bebop_control_node") {
        // bebop_autonomy parameters
        this->declare_parameter<double>("max_tilt_deg");
        this->declare_parameter<double>("max_vert_speed");

        // PID gains
        this->declare_parameter<double>("kp_xy");
        this->declare_parameter<double>("ki_xy");
        this->declare_parameter<double>("kd_xy");

        // Dirty derivative for acceleration control for P control
        double cutoff_frequency = this->declare_parameter<double>("cutoff_frequency"); // Hz
        double update_rate = this->declare_parameter<double>("update_rate"); // Hz
        dirty_N_ = cutoff_frequency * 2.0 * M_PI; // Convert to rad/s
        dt_ = 1.0 / update_rate;
        dirty_ = std::make_unique<bebop_control::DirtyDerivative>(dirty_N_, dt_);

        // Topics
        this->declare_parameter<std::string>("odom_topic");
        this->declare_parameter<std::string>("des_vel_topic");
        this->declare_parameter<std::string>("cmd_vel_topic");
        this->declare_parameter<std::string>("bebop_mode_topic");
        
        // Load parameters
        max_tilt_rad_ = this->get_parameter("max_tilt_deg").as_double() * M_PI / 180.0;
        max_vert_speed_ = this->get_parameter("max_vert_speed").as_double();

        // Load gains
        kp_xy_ = this->get_parameter("kp_xy").as_double();
        ki_xy_ = this->get_parameter("ki_xy").as_double();
        kd_xy_ = this->get_parameter("kd_xy").as_double();

        // Subscribers
        // Odometry
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            std::bind(&BebopControlNode::odomCallback, this, std::placeholders::_1));
        // Desired velocity
        std::string des_vel_topic = this->get_parameter("des_vel_topic").as_string();
        des_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            des_vel_topic,10,
            std::bind(&BebopControlNode::desVelCallback, this, std::placeholders::_1));
        // Bebop mode (int)
        std::string bebop_mode_topic = this->get_parameter("bebop_mode_topic").as_string();
        bebop_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            bebop_mode_topic,10,
            std::bind(&BebopControlNode::bebopModeCallback, this, std::placeholders::_1));
        
        // Publishers
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        
        // Log
        RCLCPP_INFO(this->get_logger(), "Low-level Bebop controller active. Listening for %s", des_vel_topic.c_str());
    }

private:
    // State
    geometry_msgs::msg::Twist target_vel_world_; // Desired world frame vel
    Eigen::Vector3d current_vel_body_; // Current body frame vel
    Eigen::Quaterniond current_att_; // Orientation
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_cmd_time_;
    bool odom_received_ = false;
    bool cmd_received_ = false;

    // PID integral, derivative terms
    double err_sum_x_ = 0.0;
    double err_sum_y_ = 0.0;
    // Integrator Booleans
    bool integrator_on_x_ = true;
    bool integrator_on_y_ = true;
    bool is_saturated_x_ = false;
    bool is_saturated_y_ = false;
    int sgn_error_x_, sgn_error_y_;
    int sgn_u_pitch, sgn_u_roll;

    // Parameters
    double max_tilt_rad_;
    double max_vert_speed_;
    double kp_xy_, ki_xy_, kd_xy_;
    int bebop_mode_ = 0;
    double dt_;
    double dirty_N_;

    void desVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_vel_world_ = *msg;
        last_cmd_time_ = this->now();
        cmd_received_ = true;
        controlLoop();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Check for finite values
        if ( !std::isfinite(msg->twist.twist.linear.x) || 
             !std::isfinite(msg->twist.twist.linear.y) || 
             !std::isfinite(msg->twist.twist.linear.z) || 
             !std::isfinite(msg->pose.pose.orientation.w) ) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),1000,"Received non-finite Odometry message");
            return;
        }
        
        last_odom_time_ = this->now();
        odom_received_ = true;

        if (bebop_mode_ == 1 && cmd_received_)
        {
            double time_since_cmd = (this->now() - last_cmd_time_).seconds();
            if (time_since_cmd > 1.0)
            {
                stopDrone();
                RCLCPP_ERROR_THROTTLE(this->get_logger(),*this->get_clock(),1000,"No recent command");
            }
        }

        // Store body frame vel
        current_vel_body_ = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        // Store attitude
        current_att_ = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
    }

    void bebopModeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Check if we are switching from teleop (0) into offboard (1)
        if (bebop_mode_ != 1 && msg->data == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD, freshening control time stamp");
            last_cmd_time_ = this->now();
            // Reset integral terms
            err_sum_x_ = 0.0;
            err_sum_y_ = 0.0;
        }
        bebop_mode_ = msg->data;
    }

    void controlLoop() {
        // Hover if no odom (either never received or stale)
        if (!odom_received_ || (this->now() - last_odom_time_).seconds() > 0.5) {
            stopDrone();
            RCLCPP_WARN(this->get_logger(), "Stale odometry.");
            return;
        }

        // Hover if not in offboard mode (mode 1)
        if (bebop_mode_ != 1) {
            // Reset integral terms
            err_sum_x_ = 0.0;
            err_sum_y_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "Not in offboard mode.");
            return;
        }

        // Coordinate transform
        // world -> body
        current_att_.normalize();

        // Convert target vel to body frame
        Eigen::Vector3d target_vel_body = current_att_.inverse() * Eigen::Vector3d(
            target_vel_world_.linear.x,
            target_vel_world_.linear.y,
            target_vel_world_.linear.z
        );

        // Calculate errors (body frame)
        double err_x = target_vel_body.x() - current_vel_body_.x();
        double err_y = target_vel_body.y() - current_vel_body_.y();
        sgn_error_x_ = (err_x > 0) ? 1 : ((err_x < 0) ? -1 : 0);
        sgn_error_y_ = (err_y > 0) ? 1 : ((err_y < 0) ? -1 : 0);

        // Integral
        if (integrator_on_x_) {
            err_sum_x_ = err_sum_x_ + err_x * dt_;
        }
        
        if (integrator_on_y_) {
            err_sum_y_ = err_sum_y_ + err_y * dt_;
        }
        
        // Derivative
        // elements are x, y, z accelerations in body frame
        dirty_->propagate(current_vel_body_);
        Eigen::Vector3d acceleration_estimate = dirty_->get_velocity_estimate(); 
        
        // PID output (desired tilt in rad)
        // Normalized wrt max values
        double u_pitch = (kp_xy_ * err_x + ki_xy_ * err_sum_x_ - kd_xy_ * acceleration_estimate.x());
        double u_roll  = (kp_xy_ * err_y + ki_xy_ * err_sum_y_ - kd_xy_ * acceleration_estimate.y());
        sgn_u_pitch = (u_pitch > 0) ? 1 : ((u_pitch < 0) ? -1 : 0);
        sgn_u_roll = (u_roll > 0) ? 1 : ((u_roll < 0) ? -1 : 0);

        // Anti-windup
        bool sgn_in_matches_sgn_out_X = (sgn_error_x_ == sgn_u_pitch);
        bool sgn_in_matches_sgn_out_Y = (sgn_error_y_ == sgn_u_roll);
        // Check for zero division
        if (max_tilt_rad_ < 1e-6 || max_vert_speed_ < 1e-6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000, "Actuator limits are too small");
            stopDrone();
            return;
        }
        double u_pitch_sat = std::clamp(u_pitch / max_tilt_rad_, -1.0, 1.0);
        double u_roll_sat = std::clamp(u_roll / max_tilt_rad_, -1.0, 1.0);
        is_saturated_x_ = (std::abs(u_pitch) >= max_tilt_rad_);
        is_saturated_y_ = (std::abs(u_roll) >= max_tilt_rad_);
        integrator_on_x_ = !(is_saturated_x_ && sgn_in_matches_sgn_out_X);
        integrator_on_y_ = !(is_saturated_y_ && sgn_in_matches_sgn_out_Y);
        
        // Output
        geometry_msgs::msg::Twist cmd_vel;
        // Normalize
        cmd_vel.linear.x = u_pitch_sat;
        cmd_vel.linear.y = u_roll_sat;
        cmd_vel.linear.z = std::clamp(target_vel_body.z() / max_vert_speed_,-1.0,1.0);
        cmd_vel.angular.z = std::clamp(target_vel_world_.angular.y, -1.0,1.0); // Yaw rate command directly
        
        // Check that values are finite
        if ( !std::isfinite(cmd_vel.linear.x) || 
             !std::isfinite(cmd_vel.linear.y) || 
             !std::isfinite(cmd_vel.linear.z) || 
             !std::isfinite(cmd_vel.angular.z) ) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),1000,"Controller produced infinite cmd_vel");
            stopDrone();
        } else{
            // Publish
            cmd_vel_pub_->publish(cmd_vel);
        }
    }

    void stopDrone() {
        // Reset integral terms
        err_sum_x_ = 0.0;
        err_sum_y_ = 0.0;
        // Send zero cmd_vel
        geometry_msgs::msg::Twist cmd_vel_zero;
        cmd_vel_zero.linear.x = 0.0;
        cmd_vel_zero.linear.y = 0.0;
        cmd_vel_zero.linear.z = 0.0;
        cmd_vel_zero.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_zero);
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr des_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bebop_mode_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Helper
    std::unique_ptr<bebop_control::DirtyDerivative> dirty_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BebopControlNode>());
  rclcpp::shutdown();
  return 0;
}