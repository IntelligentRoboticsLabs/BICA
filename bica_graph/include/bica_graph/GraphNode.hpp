/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Intelligent Robotics Core S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intelligent Robotics Core nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

/* Author: Francisco Martín Rico - fmrico@gmail.com */

#ifndef BICA_GRAPH_GRAPHNODE__HPP_
#define BICA_GRAPH_GRAPHNODE__HPP_

#include <vector>
#include <map>
#include <string>
#include <memory>

#include "bica_graph/GraphInterface.hpp"
#include "bica_graph/Graph.hpp"

#include "bica_msgs/msg/graph_update.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bica_graph
{

class GraphNode : public GraphInterface
{
public:
  GraphNode(rclcpp::Node::SharedPtr provided_node);

  void add_node(const Node & node);
  void remove_node(const std::string node);
  bool exist_node(const std::string node);
  std::optional<Node> get_node(const std::string node);

  bool add_edge(const Edge & edge);

  bool remove_edge(const Edge & edge);

  bool exist_edge(const Edge & edge);

  std::optional<std::vector<Edge>*> get_edges(
    const std::string & source,
    const std::string & target);

  std::string to_string() const;
  void from_string(const std::string & graph_str);

  size_t get_num_edges() const;
  size_t get_num_nodes() const;
  
  void process_updates();

private:
  void update_callback(const bica_msgs::msg::GraphUpdate::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  Graph graph_;
  Graph chached_graph_;
  int seq_;

  rclcpp::Publisher<bica_msgs::msg::GraphUpdate>::SharedPtr update_pub_;
  rclcpp::Subscription<bica_msgs::msg::GraphUpdate>::SharedPtr update_sub_;

  std::vector<bica_msgs::msg::GraphUpdate> updates_;
  std::vector<bica_msgs::msg::GraphUpdate> pending_updates_;
  bool initialized_;
  int last_sub_count_;
  rclcpp::Time last_ts_;
};

}  // namespace plansys2

#endif  // BICA_GRAPH_GRAPHNODE__HPP_
