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

class Phase1 : public bica_behavior_tree::ActivationActionNode
{
public:
  explicit Phase1(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cerr << "CompA::Phase1 tick() " << counter_ << std::endl;

    if (counter_++ < 5) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

class Phase2 : public bica_behavior_tree::ActivationActionNode
{
public:
  explicit Phase2(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cerr << "CompA::Phase2 tick() " << counter_ << std::endl;

    if (counter_++ < 5) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

class Phase3 : public bica_behavior_tree::ActivationActionNode
{
public:
  explicit Phase3(const std::string & name)
  : bica_behavior_tree::ActivationActionNode(name), counter_(0)
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cerr << "CompA::Phase3 tick() " << counter_ << std::endl;

    if (counter_++ < 5) {
      return BT::NodeStatus::RUNNING;
    } else {
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  int counter_;
};

class CompA : public bica::Component
{
public:
  CompA()
  : bica::Component("A", 1), finished_(false)
  {
    factory_.registerNodeType<Phase1>("Phase1");
    factory_.registerNodeType<Phase2>("Phase2");
    factory_.registerNodeType<Phase3>("Phase3");
    factory_.registerFromPlugin(BT::SharedLibrary().getOSName("activation_sequence_bt_node"));
    factory_.registerFromPlugin(BT::SharedLibrary().getOSName("activation_action_bt_node"));
  }

  void on_activate()
  {
    std::string pkgpath = ament_index_cpp::get_package_share_directory("bica_examples");
    std::string xml_file = pkgpath + "/nodes_bt/node_A_tree.xml";

    tree_ = factory_.createTreeFromFile(xml_file);
    tree_.configureActivations<Phase1>("Phase1", this, {"B"});
    tree_.configureActivations<Phase2>("Phase2", this, {"D", "C"});
    tree_.configureActivations<Phase3>("Phase3", this, {"C"});
  }

  void step()
  {
    if (!finished_) {
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

  auto component = std::make_shared<CompA>();

  component->execute();

  rclcpp::shutdown();

  return 0;
}
