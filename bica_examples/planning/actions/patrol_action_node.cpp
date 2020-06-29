// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "plansys2_msgs/action/execute_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "bica/Component.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Patrol : public plansys2::ActionExecutorClient
{
public:
  Patrol()
  : plansys2::ActionExecutorClient("patrol")
  {
    bica_component_ = std::make_shared<bica::Component>("bica_patrol", 1);
    bica_component_->addDependency("B");
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    getFeedback()->progress = 0.0;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    bica_component_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    bica_component_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void actionStep()
  {
    if (getFeedback()->progress < 100.0) {
      getFeedback()->progress += 2.0;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.5;

    cmd_vel_pub_->publish(cmd);

    bica_component_->execute_once(false);
    rclcpp::spin_some(bica_component_->get_node_base_interface());
  }

  bool isFinished()
  {
    if (getFeedback()->progress >= 100.0) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.0;

      cmd_vel_pub_->publish(cmd);

      return true;
    } else {
      return false;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<bica::Component> bica_component_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
