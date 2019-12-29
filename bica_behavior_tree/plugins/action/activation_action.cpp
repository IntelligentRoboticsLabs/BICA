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


#include <stdexcept>
#include <sstream>
#include <string>
#include <set>

#include "behaviortree_cpp_v3/control_node.h"

#include "bica_behavior_tree/action/activation_action.hpp"

namespace bica_behavior_tree
{

ActivationActionNode::ActivationActionNode(const std::string & name)
: BT::ActionNodeBase(name, {}), previous_status_(BT::NodeStatus::IDLE)
{
  setRegistrationID("ActivationAction");
}


BT::NodeStatus
ActivationActionNode::tick()
{
  BT::NodeStatus previous_status_ = status();

  if (previous_status_ == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);
    previous_status_ = BT::NodeStatus::RUNNING;
  }

  return previous_status_;
}

void
ActivationActionNode::init(
  bica::Component * component,
  std::set<std::string> activations)
{
  component_ = component;
  activations_ = activations;
}

BT::NodeStatus
ActivationActionNode::onStart()
{
  for (const auto act : activations_) {
    component_->addDependency(act);
  }
}

BT::NodeStatus
ActivationActionNode::onStop()
{
  for (const auto act : activations_) {
    component_->removeDependency(act);
  }
}

}  // namespace bica_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bica_behavior_tree::ActivationActionNode>("ActivationAction");
}
