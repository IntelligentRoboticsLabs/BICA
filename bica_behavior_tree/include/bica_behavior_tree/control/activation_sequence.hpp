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

#ifndef BICA_BEHAVIOR_TREE__CONTROL__ACTIVATION_SEQUENCE_HPP_
#define BICA_BEHAVIOR_TREE__CONTROL__ACTIVATION_SEQUENCE_HPP_

#include <stdexcept>
#include <sstream>
#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bica_behavior_tree
{

class ActivationSequenceNode : public BT::ControlNode
{
public:
  explicit ActivationSequenceNode(const std::string & name);
  ~ActivationSequenceNode() override = default;

  void reset();
  void halt() override;

protected:
  BT::NodeStatus tick() override;
  size_t current_child_idx_;
};

}  // namespace bica_behavior_tree

#endif  // BICA_BEHAVIOR_TREE__CONTROL__ACTIVATION_SEQUENCE_HPP_
