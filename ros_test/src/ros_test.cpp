#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <utility>
#include <chrono>

class RosTestNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_msg_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_msg_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_msg_sub_;
  rclcpp::TimerBase::SharedPtr yaw_controller_timer_;
  rclcpp::TimerBase::SharedPtr angle_controller_timer_;

  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;

  double targetPitch_ = 0.0;
  double targetYaw_ = 0.0;
  double targetPosition_x_ = 0.0;
  double targetPosition_y_ = 0.0;


  struct PID
  {
    double kp = 20.0; // somewhat okay for angle control PID
    double kd = 1.0;
    double ki = 0.0;
  } pid_;

  struct PID_pos
  {
    double kp = 0.5; 
    double kd = 1.0;
    double ki = 0.0;
  } pid_pos_;

  struct PID_yaw
  {
    double kp = 0.5; 
    double kd = 0.0;
    double ki = 0.0;
  } pid_yaw_;

  double previousTimeUpdate;
  double prevError;
  double prevError_ang;
  double eIntegral;

  double prevError_pos;
  double eIntegral_pos;

  uint64_t sample_time = 10;
  uint64_t sample_time_pos = 10;

public:
  RosTestNode() : Node("ros_test_node")
  {
    twist_msg_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    imu_msg_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_plugin/out", 10, std::bind(&RosTestNode::ImuMsgCallback, this, std::placeholders::_1));
    odom_msg_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RosTestNode::OdomMsgCallback, this, std::placeholders::_1));
    // yaw_controller_timer_ = this->create_wall_timer(std::chrono::milliseconds(sample_time_pos), std::bind(&RosTestNode::yawLoop, this));
    angle_controller_timer_ = this->create_wall_timer(std::chrono::milliseconds(sample_time), std::bind(&RosTestNode::Controllerloop, this));

    previousTimeUpdate = this->get_clock()->now().nanoseconds();
  }

  ~RosTestNode() { RCLCPP_INFO(this->get_logger(), "Destructor called..!"); }

  void OdomMsgCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double actualPosition_x = msg->pose.pose.position.x;
    double actualPosition_y = msg->pose.pose.position.y;
    double error_pos = this->targetPosition_x_ - actualPosition_x;
    eIntegral_pos += error_pos * (static_cast<double>(this->sample_time_pos) / 1000);

    targetPitch_ = this->pid_pos_.kp * error_pos + this->pid_pos_.ki * eIntegral_pos + this->pid_pos_.kd * (error_pos - prevError_pos) / (static_cast<double>(this->sample_time_pos) / 1000);
    if (targetPitch_ >= 0.02)
    {
      targetPitch_ = 0.02;
    }
    else if (targetPitch_ <= -0.02)
    {
      targetPitch_ = -0.02;
    }

    // RCLCPP_INFO(this->get_logger(), "Pitch Setpoint: %f", this->targetPitch_);

    prevError_pos = error_pos;
  }

  void ImuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    this->roll_ = roll;
    this->pitch_ = pitch;
    this->yaw_ = yaw;

    RCLCPP_INFO(this->get_logger(), "pitch: %f", this->pitch_);
    RCLCPP_INFO(this->get_logger(), "yaw: %f", this->yaw_);
  }

  double yawLoop()
{
    double error = this->targetYaw_ - this->yaw_;

    eIntegral += error * (static_cast<double>(this->sample_time_pos) / 1000);

    double ang_speed = this->pid_yaw_.kp * error + this->pid_yaw_.ki * eIntegral + this->pid_yaw_.kd * (error - prevError) / (static_cast<double>(this->sample_time_pos) / 1000);

    prevError_ang = error;

    if(ang_speed >= 1.0)
    {
        ang_speed = 1.0;
    }
    else if(ang_speed <= -1.0)
    {
        ang_speed = -1.0;
    }

    return ang_speed;
}

  void Controllerloop()
  {
    geometry_msgs::msg::Twist twist_msg;
    double error = this->targetPitch_ - this->pitch_;

    eIntegral += error * (static_cast<double>(this->sample_time) / 1000);

    twist_msg.linear.x = this->pid_.kp * error + this->pid_.ki * eIntegral + this->pid_.kd * (error - prevError) / (static_cast<double>(this->sample_time) / 1000);

    prevError = error;

    if(twist_msg.linear.x>=0.5){
      twist_msg.linear.x = 0.5;
    }

    else if(twist_msg.linear.x<=-0.5){
      twist_msg.linear.x = -0.5;
    }

    twist_msg.angular.z = yawLoop();

    this->twist_msg_pub_->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), "vel x: %f", twist_msg.linear.x);
    RCLCPP_INFO(this->get_logger(), "vel ang z: %f", twist_msg.angular.z);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
