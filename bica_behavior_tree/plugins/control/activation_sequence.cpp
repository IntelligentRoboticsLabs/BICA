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
#include "behaviortree_cpp_v3/control_node.h"

#include "bica_behavior_tree/action/activation_action.hpp"
#include "bica_behavior_tree/control/activation_sequence.hpp"

namespace bica_behavior_tree
{

ActivationSequenceNode::ActivationSequenceNode(const std::string & name)
: BT::ControlNode(name, {}), current_child_idx_(0)
{
  setRegistrationID("ActivationSequence");
}

void
ActivationSequenceNode::reset()
{
  current_child_idx_ = 0;
}

BT::NodeStatus
ActivationSequenceNode::tick()
{
  const size_t children_count = children_nodes_.size();

  if (status() == BT::NodeStatus::IDLE) {
    BT::TreeNode * current_child_node = children_nodes_[current_child_idx_];
    if (auto control_node =
      dynamic_cast<bica_behavior_tree::ActivationActionNode *>(current_child_node))
    {
      control_node->onStart();
    }
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count) {
    BT::TreeNode * current_child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = current_child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        {
          return child_status;
        }
      case BT::NodeStatus::FAILURE:
        {
          // Reset on failure
          haltChildren(0);
          current_child_idx_ = 0;
          return child_status;
        }
      case BT::NodeStatus::SUCCESS:
        {
          if (auto control_node =
            dynamic_cast<bica_behavior_tree::ActivationActionNode *>(current_child_node))
          {
            control_node->onStop();
          }

          current_child_idx_++;

          current_child_node = children_nodes_[current_child_idx_];
          if (current_child_idx_ < children_count) {
            if (auto control_node =
              dynamic_cast<bica_behavior_tree::ActivationActionNode *>(current_child_node))
            {
              control_node->onStart();
            }
          }
        }
        break;

      case BT::NodeStatus::IDLE:
        {
          throw BT::LogicError("A child node must never return IDLE");
        }
    }   // end switch
  }  // end while loop

  // The entire while loop completed. This means that all the children returned SUCCESS.
  if (current_child_idx_ == children_count) {
    haltChildren(0);
    current_child_idx_ = 0;
  }
  return BT::NodeStatus::SUCCESS;
}

void ActivationSequenceNode::halt()
{
  current_child_idx_ = 0;
  BT::ControlNode::halt();
}

}  // namespace bica_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bica_behavior_tree::ActivationSequenceNode>("ActivationSequence");
}
