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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "axebot_control/go_to_goal.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<
    axebot_control::GoToGoal> go_to_goal_node = std::make_shared<axebot_control::GoToGoal>();
  go_to_goal_node->setAngularGain(2.0);
  go_to_goal_node->setLinearGain(0.5);
  go_to_goal_node->setGoalDistanceTolerance(0.2);
  rclcpp::spin(go_to_goal_node);
  rclcpp::shutdown();
  return 0;
}
