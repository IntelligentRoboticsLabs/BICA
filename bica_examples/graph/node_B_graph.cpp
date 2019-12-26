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

#include "bica/Component.hpp"
#include "bica_graph/TypedGraphNode.hpp"

#include "rclcpp/rclcpp.hpp"

class CompB : public bica::Component
{
public:
  CompB() 
  : bica::Component("B", 1)
  {
    addDependency("C");
    addDependency("D");

    graph_ = std::make_shared<bica_graph::TypedGraphNode>(get_name());
  }

  void on_activate()
  {
    RCLCPP_INFO(get_logger(), "Activate");
    graph_->add_node(bica_graph::Node{get_name(), "component"});
  }

  void on_deactivate()
  {
    RCLCPP_INFO(get_logger(), "DeActivate");
    graph_->remove_node(get_name());
  }

	void step() {
		RCLCPP_INFO(get_logger(), "CompB::step()");
	}

private:
  std::shared_ptr<bica_graph::TypedGraphNode> graph_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto component = std::make_shared<CompB>();
  
  component->execute();

  rclcpp::shutdown();

  return 0;
}
