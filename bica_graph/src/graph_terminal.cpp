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


#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <list>
#include <random>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bica_graph/TypedGraphNode.hpp"

std::vector<std::string> tokenize(const std::string & text)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(" ", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}


class GraphTerminal
{
public:
  explicit GraphTerminal(const std::string & id)
  {
    graph_ = std::make_shared<bica_graph::TypedGraphNode>(id);
  }

  void run_console()
  {
    std::string line;
    bool success = true;

    std::cout << "BICA Graph console. Type \"quit\" to finish" << std::endl;
    std::cout << "> ";
    while (std::getline(std::cin, line)) {
      if (line == "quit") {
        break;
      }

      process_command(line);
      std::cout << "> ";
    }

    std::cout << "Finishing..." << std::endl;
  }

  void list_nodes(void)
  {
    auto nodes = graph_->get_nodes();

    std::cout << "total nodes: " << graph_->get_num_nodes() << std::endl;
    for (const auto & node : nodes) {
      std::cout << "\t" << node.second.to_string() << std::endl;
    }
  }

  void list_edges(void)
  {
    std::cout << "total string edges: " << graph_->get_num_edges() << std::endl;
    for (const auto & pair : graph_->get_edges()) {
      for (const auto & edge : pair.second) {
        if (edge.type != "tf") {
          std::cout << "\t" << edge.to_string() << std::endl;
        } else {
          auto tf_edge = graph_->get_tf_edge(edge.source, edge.target);

          if (tf_edge) {
            tf2::Stamped<tf2::Transform> tf;
            tf2::convert(tf_edge.value().tf_, tf);

            double roll, pitch, yaw;
            tf2::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);

            std::cout << "\tedge::" << edge.source << "->" << edge.target + "::(" <<
              tf.getOrigin().x() << ", " << tf.getOrigin().y() << ", " <<
              tf.getOrigin().z() << ") [" << roll << ", " << pitch << ", " <<
              yaw << "]::tf" << std::endl;

          } else {
            std::cout << "\t" << edge.to_string() << std::endl;
          }
        }
      }
    }
  }

  void process_list(const std::vector<std::string> & command)
  {
    if (command.size() == 1) {
      list_nodes();
      list_edges();
    } else if ((command.size() == 2) && (command[1] == "nodes")) {
      list_nodes();
    } else if ((command.size() == 2) && (command[1] == "edges")) {
      list_edges();
    } else {
      std::cout << "\tUsage: list [nodes|edges]" << std::endl;
    }
  }

  void process_add(const std::vector<std::string> & command)
  {
    if (command.size() > 1) {
      if (command[1] == "node") {
        if (command.size() != 4) {
          std::cout << "\tUsage: \n\t\tadd node id type" << std::endl;
        } else {
          graph_->add_node(bica_graph::Node{command[2], command[3]});
        }
      } else if (command[1] == "edge") {
        if ((command.size() == 2) ||
          ((command.size() > 2) &&
          ((command[2] != "symbolic") && (command[2] != "tf"))))
        {
          std::cout << "\t\tadd edge [symbolic|tf] source target data" << std::endl;
        } else if (command[2] == "symbolic") {
          if (command.size() >= 6) {
            std::string data = command[5];
            for (int i = 6; i < command.size(); i++) {data = data + " " + command[i];}
            graph_->add_edge(bica_graph::Edge{data, "symbolic", command[3], command[4]});
          } else {
            std::cout << "\t\tadd edge symbolic source target data" << std::endl;
          }
        } else if (command[2] == "tf") {
          if (command.size() != 11) {
            std::cout << "Please, introduce a correct coordinate in format x  y  z y  p  r: " <<
              std::endl;
          } else {
            double x = std::stod(command[5]);
            double y = std::stod(command[6]);
            double z = std::stod(command[7]);
            double ry = std::stod(command[8]);
            double rp = std::stod(command[9]);
            double rr = std::stod(command[10]);

            tf2::Quaternion q;
            q.setRPY(rr, rp, ry);
            tf2::Transform trans(q, tf2::Vector3(x, y, z));
            bica_graph::TFEdge tf_edge;
            tf_edge.tf_.header.frame_id = command[3];
            tf_edge.tf_.child_frame_id = command[4];
            tf2::convert(trans, tf_edge.tf_.transform);

            graph_->add_tf_edge(tf_edge);
          }
        }  // if ((command[5].a ...
      } else {
        std::cout << "\tUsage: \n\t\tadd edge [symbolic|tf]..." << std::endl;
      }  // else if (command[2] == "tf") ...
    } else {
      std::cout << "\tUsage: \n\t\tadd [node|edge]..." << std::endl;
    }
  }


  void process_remove(const std::vector<std::string> & command)
  {
    if (command.size() > 1) {
      if (command[1] == "node") {
        if (command.size() != 3) {
          std::cout << "\tUsage: \n\t\tremove node id" << std::endl;
        } else {
          graph_->remove_node(command[2]);
        }
      } else if (command[1] == "edge") {
        if ((command.size() == 2) ||
          ((command.size() > 2) &&
          ((command[2] != "symbolic") && (command[2] != "tf"))))
        {
          std::cout << "\t\tremove edge [symbolic|tf] source target [data]" << std::endl;
        } else if (command[2] == "symbolic") {
          if (command.size() >= 6) {
            std::string data = command[5];
            for (int i = 6; i < command.size(); i++) {data = data + " " + command[i];}
            graph_->remove_edge(bica_graph::Edge{data, "symbolic", command[3], command[4]});
          } else {
            std::cout << "\t\tremove edge symbolic source target data" << std::endl;
          }
        } else if (command[2] == "tf") {
          if (command.size() == 5) {
            graph_->remove_edge(bica_graph::Edge{"", "tf", command[3], command[4]});
          } else {
            std::cout << "\t\tremove edge tf source target" << std::endl;
          }
        } else {
          std::cout << "\t\tremove edge [symbolic|tf] source target [data]" << std::endl;
        }
      } else {
        std::cout << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
    }
  }

  void process_command(const std::string & command)
  {
    std::vector<std::string> tokens = tokenize(command);

    if (tokens.empty()) {
      return;
    }

    if (tokens[0] == "list") {
      process_list(tokens);
    } else if (tokens[0] == "add") {
      process_add(tokens);
    } else if (tokens[0] == "remove") {
      process_remove(tokens);
    } else {
      std::cout << "Command not found" << std::endl;
    }
  }

private:
  std::shared_ptr<bica_graph::TypedGraphNode> graph_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 10000);

  GraphTerminal terminal("graph_terminal" + std::to_string(dis(gen)));
  terminal.run_console();

  rclcpp::shutdown();

  return 0;
}
