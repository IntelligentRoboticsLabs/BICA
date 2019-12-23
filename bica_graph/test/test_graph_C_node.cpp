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
#include <random>

#include "bica_graph/GraphNode.hpp"
#include "bica_graph/Types.hpp"
#include "rclcpp/rclcpp.hpp"

class NodeC : public rclcpp::Node
{
public:
  NodeC()
  : rclcpp::Node("node_C"), gen_(rd_()), dis_(1, 10)
  {
    graph_ = std::make_shared<bica_graph::GraphNode>("node_C");
  }

  void do_work()
  {
    //for (int i = 0; i < dis_(gen_); i++) 
    {
      int new_value = dis_(gen_);

      if (new_value > 3 && new_value <= 5) {
        std::string name = "node_" + std::to_string(dis_(gen_));
        std::string type = "type_" + std::to_string(dis_(gen_));
       
        bica_graph::Node node{name, type};

        graph_->add_node(node);
      } else if (new_value == 6) {
        std::string name = "node_" + std::to_string(dis_(gen_));

        graph_->remove_node(name);
      } else if (new_value >= 7 && new_value <= 9) {
        std::string source = "node_" + std::to_string(dis_(gen_));
        std::string target = "node_" + std::to_string(dis_(gen_));
        std::string type = "type_" + std::to_string(dis_(gen_));
        std::string content = "content_" + std::to_string(dis_(gen_));
        
        bica_graph::Edge edge{content, type, source, target};

        graph_->add_edge(edge);
      } else if (new_value == 10) {
        std::string source = "node_" + std::to_string(dis_(gen_));
        std::string target = "node_" + std::to_string(dis_(gen_));
        std::string type = "type_" + std::to_string(dis_(gen_));
        std::string content = "content_" + std::to_string(dis_(gen_));

        bica_graph::Edge edge{content, type, source, target};

        graph_->remove_edge(edge);
      }
    }
  }

  std::string get()
  {
    return graph_->to_string();
  }

private:
  std::shared_ptr<bica_graph::GraphNode> graph_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<> dis_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeC>();

  rclcpp::Rate rate(1);

  while (rclcpp::ok()) {
    node->do_work();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
