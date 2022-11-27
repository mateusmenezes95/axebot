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

#ifndef AXEBOT_CONTROL__GO_TO_GOAL_HPP_
#define AXEBOT_CONTROL__GO_TO_GOAL_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace axebot_control {

class GoToGoal : public rclcpp::Node {
 public:
  GoToGoal();
  ~GoToGoal();
  void setAngularGain(double gain);
  void setLinearGain(double gain);
  void setGoalDistanceTolerance(double tolerance);

 protected:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

 private:
  void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void readGoalPointCallback(const geometry_msgs::msg::Vector3::SharedPtr goal);

  geometry_msgs::msg::Twist cmd_vel_;
  tf2::Vector3 goal_;
  bool no_goal_received_;
  double angular_gain_;
  double linear_gain_;
  double tolerance_;
};

}  // namespace axebot_control

#endif  // AXEBOT_CONTROL__GO_TO_GOAL_HPP_
