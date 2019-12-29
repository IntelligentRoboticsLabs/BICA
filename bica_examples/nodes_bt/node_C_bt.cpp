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
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "bica_behavior_tree/action/activation_action.hpp"
#include "bica_behavior_tree/BicaTree.hpp"

#include "bica/Component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class Phase1C : public bica_behavior_tree::ActivationActionNode
{
public:
  explicit Phase1C(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cerr << "CompC::Phase1C tick() " << counter_ << std::endl;

    if (counter_++ < 2) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

class Phase2C : public bica_behavior_tree::ActivationActionNode
{
public:
  explicit Phase2C(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cerr << "CompC::Phase2C tick() " << counter_ << std::endl;

    if (counter_++ < 2) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};


class CompC : public bica::Component
{
public:
  CompC()
  : bica::Component("C", 10), finished_(false)
  {
    factory_.registerNodeType<Phase1C>("Phase1C");
    factory_.registerNodeType<Phase2C>("Phase2C");
    factory_.registerFromPlugin(BT::SharedLibrary().getOSName("activation_sequence_bt_node"));
    factory_.registerFromPlugin(BT::SharedLibrary().getOSName("activation_action_bt_node"));
  }

  void on_activate()
  {
    std::string pkgpath = ament_index_cpp::get_package_share_directory("bica_examples");
    std::string xml_file = pkgpath + "/nodes_bt/node_C_tree.xml";

    tree_ = factory_.createTreeFromFile(xml_file);
    tree_.configureActivations<Phase1C>("Phase1C", this, {});
    tree_.configureActivations<Phase2C>("Phase2C", this, {});
  }

  void step()
  {
    if (!finished_) {
      finished_ = tree_.root_node->executeTick() == BT::NodeStatus::SUCCESS;
    } else {
      on_activate();  // cyclic execution
      finished_ = tree_.root_node->executeTick() == BT::NodeStatus::SUCCESS;
    }
  }

private:
  BT::BehaviorTreeFactory factory_;
  bica_behavior_tree::BicaTree tree_;
  bool finished_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto component = std::make_shared<CompC>();

  while (rclcpp::ok()) {
    component->execute_once();
  }

  rclcpp::shutdown();

  return 0;
}
