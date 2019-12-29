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

#ifndef BICA_BEHAVIOR_TREE__BICATREE_HPP_
#define BICA_BEHAVIOR_TREE__BICATREE_HPP_

#include <utility>
#include <string>
#include <set>

#include "bica/Component.hpp"

#include "bica_behavior_tree/action/activation_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bica_behavior_tree
{

struct BicaTree : BT::Tree
{
  BicaTree & operator=(Tree && other)
  {
    root_node = std::move(other.root_node);
    nodes = std::move(other.nodes);
    blackboard_stack = std::move(other.blackboard_stack);
    manifests = std::move(other.manifests);
    return *this;
  }

  template<typename T>
  void configureActivations(
    const std::string & ID,
    bica::Component * component,
    std::set<std::string> activations)
  {
    for (auto & node : nodes) {
      if (auto action = dynamic_cast<T *>(node.get())) {
        if (action->name() == ID) {
          action->init(component, activations);
        }
      }
    }
  }
};

}  // namespace bica_behavior_tree

#endif  // BICA_BEHAVIOR_TREE__BICATREE_HPP_
