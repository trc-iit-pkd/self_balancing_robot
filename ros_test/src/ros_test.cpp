#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"


#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <utility>
#include <chrono>


class RosTestNode: public rclcpp::Node
{

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_msg_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_msg_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;

  double targetPitch_ = 0.0;

  struct PID{
    double kp = 6.0; // somewhat okay for angle control PID
    double kd = 0.0005;
    double ki = 3.0;
  }pid_;

  double previousTimeUpdate;
  double prevError;
  double eIntegral;

  uint64_t sample_time = 100;


public:
  RosTestNode(): Node("ros_test_node"){
      twist_msg_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      imu_msg_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&RosTestNode::ImuMsgCallback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(std::chrono::milliseconds(sample_time), std::bind(&RosTestNode::Controllerloop, this));

      previousTimeUpdate = this->get_clock()->now().nanoseconds();

  }

  ~RosTestNode(){RCLCPP_INFO(this->get_logger(), "Destructor called..!");}

  void ImuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    this->roll_ = roll;
    this->pitch_ = pitch;
    this->yaw_ = yaw;

    // RCLCPP_INFO(this->get_logger(), "pitch: %f", this->pitch_);

  }

  void Controllerloop(){

      // Balancing bot 
      geometry_msgs::msg::Twist twist_msg;
      double sampleTime = static_cast<double>(this->sample_time)/1000;

      double currentTime = this->get_clock()->now().nanoseconds();
      double dt = (currentTime-this->previousTimeUpdate)*(1.0e-9);

      RCLCPP_INFO(this->get_logger(), "timeStep: %f", dt);
      double error = -(this->targetPitch_-this->pitch_); // since diff drive plugin has the kinematics (pitching pwd means the bot needs +ve cmd and its negation of the pitch back)
      eIntegral += error*sampleTime;
      
      // pid on the body-x velc (bot frame)
      twist_msg.linear.x = this->pid_.kp*(error) + this->pid_.kd*(error-prevError)/(sampleTime) + this->pid_.ki*(eIntegral);
      
      prevError = error;
      previousTimeUpdate = currentTime;

      this->twist_msg_pub_->publish(twist_msg);
  }

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}