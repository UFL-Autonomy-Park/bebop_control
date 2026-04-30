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

using namespace std::chrono_literals;

using TwistMsg = geometry_msgs::msg::Twist;
using OdomMsg = nav_msgs::msg::Odometry;

namespace {
    template <typename T> constexpr int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}

class BebopControlNode : public rclcpp::Node {
public:
    BebopControlNode() : Node("bebop_control_node") {
        this->declare_parameter<double>("max_tilt_angle");
        this->declare_parameter<double>("max_vertical_speed");
        this->declare_parameter<double>("kp_xy");
        this->declare_parameter<double>("ki_xy");
        this->declare_parameter<double>("control_freq_hz");

        kp_xy_ = this->get_parameter("kp_xy").as_double();
        ki_xy_ = this->get_parameter("ki_xy").as_double();
        double control_freq_hz_ = this->get_parameter("control_freq_hz").as_double();
        double max_tilt_angle_deg_ = this->get_parameter("max_tilt_angle").as_double();
        max_vertical_speed_mps_ = this->get_parameter("max_vertical_speed").as_double();

        dt_ = 1.0 / control_freq_hz_;
        max_tilt_angle_rad_ = max_tilt_angle_deg_ * M_PI / 180.0;

        odom_sub_ = this->create_subscription<OdomMsg>(
            "filtered_odom",
            rclcpp::SensorDataQoS(),
            std::bind(&BebopControlNode::odomCallback, this, std::placeholders::_1)
        );
        des_vel_sub_ = this->create_subscription<TwistMsg>(
            "cmd_vel_des",
            10,
            std::bind(&BebopControlNode::desVelCallback, this, std::placeholders::_1)
        );
        bebop_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode",
            10,
            std::bind(&BebopControlNode::bebopModeCallback, this, std::placeholders::_1)
        );
        cmd_vel_pub_ = this->create_publisher<TwistMsg>(
            "cmd_vel",
            10
        );

        RCLCPP_INFO_STREAM(this->get_logger(), "Low-level Bebop controller active. Listening for cmd_vel");
    }

private:
    static constexpr int LOG_THROTTLE_DURATION_MS = 1000;
    static constexpr double STALE_COMMAND_TIMEOUT_SEC = 1.0;
    static constexpr double STALE_ODOM_TIMEOUT_SEC = 0.5;

    enum class FlightMode { TELEOP = 0, OFFBOARD = 1 };

    // State
    TwistMsg target_vel_mocap_; // Desired veloicty in the world/mocap frame
    Eigen::Vector3d current_vel_body_; // Current vel in the body frame
    Eigen::Quaterniond rot_body_to_mocap_; // Orientation in the world/mocap frame
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_cmd_time_;
    bool odom_received_ = false;
    bool cmd_received_ = false;

    double err_sum_x_ = 0.0;
    double err_sum_y_ = 0.0;
    bool integrator_on_x_ = true;
    bool integrator_on_y_ = true;
    bool is_saturated_x_ = false;
    bool is_saturated_y_ = false;

    double max_tilt_angle_rad_;
    double max_vertical_speed_mps_;
    double kp_xy_, ki_xy_;
    double dt_;
    FlightMode current_mode_ = FlightMode::TELEOP;

    rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
    rclcpp::Subscription<TwistMsg>::SharedPtr des_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bebop_mode_sub_;
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;

    void controlLoop() {
        if (!odom_received_ || (this->now() - last_odom_time_) >= rclcpp::Duration::from_seconds(STALE_ODOM_TIMEOUT_SEC)) {
            stopDrone();
            RCLCPP_WARN(this->get_logger(), "Stale odometry.");
            return;
        }

        if (current_mode_ != FlightMode::OFFBOARD) {
            err_sum_x_ = 0.0;
            err_sum_y_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "Not in offboard mode.");
            return;
        }

        // Active transform from world/mocap frame to body frame
        rot_body_to_mocap_.normalize(); 
        Eigen::Vector3d target_vel_body = rot_body_to_mocap_.inverse() * Eigen::Vector3d(
            target_vel_mocap_.linear.x,
            target_vel_mocap_.linear.y,
            target_vel_mocap_.linear.z
        );

        // Calculate errors (body frame)
        double err_x = target_vel_body.x() - current_vel_body_.x();
        double err_y = target_vel_body.y() - current_vel_body_.y();
        int sgn_error_x = sgn(err_x);
        int sgn_error_y = sgn(err_y);

        // Integral (assumes a ZOH)
        if (integrator_on_x_) {
            err_sum_x_ = err_sum_x_ + err_x * dt_;
        }
        
        if (integrator_on_y_) {
            err_sum_y_ = err_sum_y_ + err_y * dt_;
        }
        
        // PI output (desired tilt in rad)
        double u_pitch = (kp_xy_ * err_x + ki_xy_ * err_sum_x_);
        double u_roll  = (kp_xy_ * err_y + ki_xy_ * err_sum_y_);
        int sgn_u_pitch = sgn(u_pitch);
        int sgn_u_roll = sgn(u_roll);

        // Anti-windup
        bool sgn_in_matches_sgn_out_X = (sgn_error_x == sgn_u_pitch);
        bool sgn_in_matches_sgn_out_Y = (sgn_error_y == sgn_u_roll);
        double u_pitch_sat = std::clamp(u_pitch / max_tilt_angle_rad_, -1.0, 1.0);
        double u_roll_sat = std::clamp(u_roll / max_tilt_angle_rad_, -1.0, 1.0);
        is_saturated_x_ = (std::abs(u_pitch) >= max_tilt_angle_rad_);
        is_saturated_y_ = (std::abs(u_roll) >= max_tilt_angle_rad_);
        integrator_on_x_ = !(is_saturated_x_ && sgn_in_matches_sgn_out_X);
        integrator_on_y_ = !(is_saturated_y_ && sgn_in_matches_sgn_out_Y);
        
        TwistMsg cmd_vel;
        cmd_vel.linear.x = u_pitch_sat;
        cmd_vel.linear.y = u_roll_sat;
        cmd_vel.linear.z = std::clamp(target_vel_body.z() / max_vertical_speed_mps_,-1.0, 1.0);
        cmd_vel.angular.z = std::clamp(target_vel_mocap_.angular.y, -1.0, 1.0); 
        
        if ( !std::isfinite(cmd_vel.linear.x) || 
             !std::isfinite(cmd_vel.linear.y) || 
             !std::isfinite(cmd_vel.linear.z) || 
             !std::isfinite(cmd_vel.angular.z) ) 
        {
            RCLCPP_FATAL(this->get_logger(), "Controller produced infinite cmd_vel");
            throw std::runtime_error("Controller produced infinite cmd_vel");
        } else {
            cmd_vel_pub_->publish(cmd_vel);
        }
    }

    void stopDrone() {
        err_sum_x_ = 0.0;
        err_sum_y_ = 0.0;
        cmd_vel_pub_->publish(TwistMsg());
    }

    void desVelCallback(const TwistMsg::SharedPtr msg) {
        last_cmd_time_ = this->now();
        cmd_received_ = true;

        target_vel_mocap_ = *msg;
        controlLoop();
    }

    void odomCallback(const OdomMsg::SharedPtr msg) {
        last_odom_time_ = this->now();
        odom_received_ = true;

        if ( !std::isfinite(msg->twist.twist.linear.x) || 
             !std::isfinite(msg->twist.twist.linear.y) || 
             !std::isfinite(msg->twist.twist.linear.z) || 
             !std::isfinite(msg->pose.pose.orientation.x) ||
             !std::isfinite(msg->pose.pose.orientation.y) ||
             !std::isfinite(msg->pose.pose.orientation.z) ||
             !std::isfinite(msg->pose.pose.orientation.w) ) 
        {
            RCLCPP_FATAL(this->get_logger(), "Received infinity/NaN in odometry message. Crashing.");
            throw std::runtime_error("Received infinity/NaN in odometry message");
        }
        
        if (current_mode_ == FlightMode::OFFBOARD && cmd_received_)
        {
            if ((this->now() - last_cmd_time_) >= rclcpp::Duration::from_seconds(STALE_COMMAND_TIMEOUT_SEC))
            {
                stopDrone();
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    LOG_THROTTLE_DURATION_MS,
                    "No recent command"
                );
            }
        }

        current_vel_body_ = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );
        rot_body_to_mocap_ = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
    }

    void bebopModeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (current_mode_ == FlightMode::TELEOP && msg->data == static_cast<int>(FlightMode::OFFBOARD))
        {
            RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD, freshening control time stamp");
            last_cmd_time_ = this->now();
            err_sum_x_ = 0.0;
            err_sum_y_ = 0.0;
        }
        current_mode_ = static_cast<FlightMode>(msg->data);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopControlNode>());
    rclcpp::shutdown();
    return 0;
}