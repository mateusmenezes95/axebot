// MIT License

// Copyright (c) 2022 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "axebot_control/go_to_goal.hpp"

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2/utils.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

namespace axebot_control {

GoToGoal::GoToGoal()
  : Node("go_to_goal")
  , no_goal_received_(false)
  , angular_gain_(1.0)
  , linear_gain_(1.0)
  , tolerance_(0.1) {
  goal_.setZero();

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/gazebo_ground_truth/odom", 1, std::bind(&GoToGoal::readOdometryCallback, this, _1));

  goal_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "~/goal", 1, std::bind(&GoToGoal::readGoalPointCallback, this, _1));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/omnidirectional_controller/cmd_vel_unstamped", 10);

  RCLCPP_INFO(this->get_logger(), "Node %s has started", this->get_logger().get_name());
}

void GoToGoal::setAngularGain(double gain) {
  this->angular_gain_ = gain;
  RCLCPP_INFO(this->get_logger(), "Angular gain set to %.2f", this->angular_gain_);
}

void GoToGoal::setLinearGain(double gain) {
  this->linear_gain_ = gain;
  RCLCPP_INFO(this->get_logger(), "Linear gain set to %.2f", this->linear_gain_);
}

void GoToGoal::setGoalDistanceTolerance(double tolerance) {
  this->tolerance_ = tolerance;
  RCLCPP_INFO(this->get_logger(), "Distance to goal tolerance set to %.2f", this->tolerance_);
}

void GoToGoal::readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  cmd_vel_.linear.x = 0;
  cmd_vel_.angular.z = 0;

  if (!no_goal_received_) {
    cmd_vel_publisher_->publish(cmd_vel_);
    return;
  }

  tf2::Vector3 robot_position;
  robot_position.setX(msg->pose.pose.position.x);
  robot_position.setY(msg->pose.pose.position.y);
  robot_position.setZ(0);

  double distance_to_goal = this->goal_.distance(robot_position);

  if (distance_to_goal < tolerance_) {
    cmd_vel_publisher_->publish(cmd_vel_);
    return;
  }

  cmd_vel_.linear.x = linear_gain_*distance_to_goal;
  cmd_vel_.angular.z = angular_gain_*(atan2(goal_.getY() - robot_position.getY(),
    goal_.getX() - robot_position.getX()) - tf2::getYaw(msg->pose.pose.orientation));

  cmd_vel_publisher_->publish(cmd_vel_);
}

void GoToGoal::readGoalPointCallback(const geometry_msgs::msg::Vector3::SharedPtr goal) {
  this->goal_.setX(goal->x);
  this->goal_.setY(goal->y);
  this->goal_.setZ(0);
  no_goal_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Receiveid goal point: (%.2f, %.2f)",
    goal_.getX(), goal_.getY());
}

GoToGoal::~GoToGoal() {}

}  // namespace axebot_control
