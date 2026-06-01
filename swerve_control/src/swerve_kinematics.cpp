// File: ~/ros2_ws/src/swerve_robot/swerve_control/src/swerve_kinematics.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <algorithm>

class SwerveKinematics : public rclcpp::Node {
public:
    SwerveKinematics() : Node("swerve_kinematics"), odom_x_(0.0), odom_y_(0.0), odom_theta_(0.0) {
        
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("offset_x", 0.3);
        this->declare_parameter("offset_y", 0.3);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        offset_x_ = this->get_parameter("offset_x").as_double();
        offset_y_ = this->get_parameter("offset_y").as_double();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SwerveKinematics::cmdVelCallback, this, std::placeholders::_1));
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&SwerveKinematics::jointStateCallback, this, std::placeholders::_1));

        steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "steering_controller/commands", 10);
        
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "velocity_controller/commands", 10);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        current_steering_angles_.resize(4, 0.0);
        current_wheel_velocities_.resize(4, 0.0);
        last_time_ = this->now();
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<std::string> steer_joints = {
            "fl_steering_joint", "fr_steering_joint", 
            "rl_steering_joint", "rr_steering_joint"
        };
        std::vector<std::string> wheel_joints = {
            "fl_wheel_joint", "fr_wheel_joint", 
            "rl_wheel_joint", "rr_wheel_joint"
        };

        bool joints_updated = false;

        for (size_t i = 0; i < 4; ++i) {
            auto it_steer = std::find(msg->name.begin(), msg->name.end(), steer_joints[i]);
            if (it_steer != msg->name.end()) {
                current_steering_angles_[i] = msg->position[std::distance(msg->name.begin(), it_steer)];
                joints_updated = true;
            }

            auto it_wheel = std::find(msg->name.begin(), msg->name.end(), wheel_joints[i]);
            if (it_wheel != msg->name.end()) {
                current_wheel_velocities_[i] = msg->velocity[std::distance(msg->name.begin(), it_wheel)];
            }
        }

        if (joints_updated) {
            calculateForwardKinematics();
        }
    }

    void calculateForwardKinematics() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = current_time;

        double vx_sum = 0.0, vy_sum = 0.0, omega_sum = 0.0;
        
        double x_pos[4] = {offset_x_, offset_x_, -offset_x_, -offset_x_};
        double y_pos[4] = {offset_y_, -offset_y_, offset_y_, -offset_y_};

        for (int i = 0; i < 4; ++i) {
            double v_wheel = current_wheel_velocities_[i] * wheel_radius_;
            double theta = current_steering_angles_[i];
            
            double v_xi = v_wheel * std::cos(theta);
            double v_yi = v_wheel * std::sin(theta);

            vx_sum += v_xi;
            vy_sum += v_yi;

            double r_sq = x_pos[i]*x_pos[i] + y_pos[i]*y_pos[i];
            omega_sum += (v_yi * x_pos[i] - v_xi * y_pos[i]) / r_sq;
        }

        double robot_vx = vx_sum / 4.0;
        double robot_vy = vy_sum / 4.0;
        double robot_omega = omega_sum / 4.0;

        double delta_x = (robot_vx * std::cos(odom_theta_) - robot_vy * std::sin(odom_theta_)) * dt;
        double delta_y = (robot_vx * std::sin(odom_theta_) + robot_vy * std::cos(odom_theta_)) * dt;
        double delta_theta = robot_omega * dt;

        odom_x_ += delta_x;
        odom_y_ += delta_y;
        odom_theta_ += delta_theta;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom_theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = robot_vx;
        odom_msg.twist.twist.linear.y = robot_vy;
        odom_msg.twist.twist.angular.z = robot_omega;

        odom_pub_->publish(odom_msg);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double omega = msg->angular.z;

        double wx[4] = {vx - omega * offset_y_, vx - omega * (-offset_y_), vx - omega * offset_y_, vx - omega * (-offset_y_)};
        double wy[4] = {vy + omega * offset_x_, vy + omega * offset_x_, vy + omega * (-offset_x_), vy + omega * (-offset_x_)};

        std_msgs::msg::Float64MultiArray steering_msg;
        std_msgs::msg::Float64MultiArray velocity_msg;

        for (size_t i = 0; i < 4; ++i) {
            double target_speed = std::hypot(wx[i], wy[i]);
            double target_angle = std::atan2(wy[i], wx[i]);

            if (std::abs(target_speed) < 0.01) {
                steering_msg.data.push_back(current_steering_angles_[i]);
                velocity_msg.data.push_back(0.0);
            } else {
                optimizeModule(target_angle, target_speed, current_steering_angles_[i]);
                target_speed = target_speed / wheel_radius_;
                steering_msg.data.push_back(target_angle);
                velocity_msg.data.push_back(target_speed);
            }
        }

        steering_pub_->publish(steering_msg);
        velocity_pub_->publish(velocity_msg);
    }

    void optimizeModule(double &target_angle, double &target_speed, double current_angle) {
        double error = target_angle - current_angle;
        error = std::atan2(std::sin(error), std::cos(error));

        if (std::abs(error) > (M_PI / 2.0 + 0.1)) {
            target_speed = -target_speed;
            error -= std::copysign(M_PI, error);
        }
        
        target_angle = current_angle + error;
        target_speed = target_speed * std::cos(error);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double wheel_radius_;
    double offset_x_;
    double offset_y_;
    
    std::vector<double> current_steering_angles_;
    std::vector<double> current_wheel_velocities_;
    
    rclcpp::Time last_time_;
    double odom_x_, odom_y_, odom_theta_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwerveKinematics>());
    rclcpp::shutdown();
    return 0;
}