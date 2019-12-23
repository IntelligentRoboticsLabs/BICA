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
#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>

#include "bica_graph/Graph.hpp"
#include "bica_graph/GraphNode.hpp"
#include "bica_graph/TypedGraphNode.hpp"
#include "bica_graph/Types.hpp"

#include "gtest/gtest.h"


TEST(bica_graph, graph_operations)
{
  bica_graph::Graph graph;

  ASSERT_FALSE(graph.exist_node("r2d2"));
  graph.add_node(bica_graph::Node{"r2d2", "robot"});
  ASSERT_TRUE(graph.exist_node("r2d2"));
  ASSERT_FALSE(graph.exist_node("kitchen"));
  graph.add_node(bica_graph::Node{"kitchen", "room"});
  ASSERT_TRUE(graph.exist_node("kitchen"));

  graph.add_node(bica_graph::Node{"room1", "room"});
  graph.add_node(bica_graph::Node{"room2", "room"});

  graph.remove_node("r2d2");
  ASSERT_FALSE(graph.exist_node("r2d2"));
  graph.add_node(bica_graph::Node{"r2d2", "robot"});
  ASSERT_TRUE(graph.exist_node("r2d2"));

  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.exist_edge(bica_graph::Edge{"is", "symbolic","r2d2", "kitchen"}));
  ASSERT_TRUE(graph.remove_edge(bica_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_FALSE(graph.exist_edge(bica_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"is_near", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"is_verynear", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"is_verynear_very", "sympedal", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"related", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(bica_graph::Edge{"related", "metric", "kitchen", "r2d2"}));

  ASSERT_EQ(graph.get_node_names_by_id("room[[:alnum:]_]*").size(), 2);
  ASSERT_EQ(graph.get_node_names_by_id("kitchen").size(), 1);
  ASSERT_EQ(graph.get_node_names_by_type("room").size(), 3);
  ASSERT_EQ(graph.get_node_names_by_type("robot").size(), 1);
  ASSERT_EQ(graph.get_edges_from_node("kitchen").size(), 5);
  ASSERT_EQ(graph.get_edges_from_node("kitchen", "symbolic").size(), 3);
  ASSERT_EQ(graph.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*").size(), 3);
  ASSERT_EQ(graph.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*", "symbolic").size(), 2);
  ASSERT_EQ(graph.get_edges_by_data("is[[:alnum:]_]*").size(), 4);

  ASSERT_EQ(graph.get_num_edges(), 6);
  graph.remove_node("r2d2");
  ASSERT_EQ(graph.get_num_edges(), 0);


  std::string graph_str = graph.to_string();
  bica_graph::Graph graph2;
  graph2.from_string(graph_str);
  std::string graph2_str = graph2.to_string();

  ASSERT_EQ(graph_str, graph2_str);
}

TEST(bica_graph, graph_node_operations)
{
  bica_graph::GraphNode graph_1("graph_test_node_1");
  bica_graph::GraphNode graph_2("graph_test_node_2");

  ASSERT_FALSE(graph_1.exist_node("r2d2"));
  graph_1.add_node(bica_graph::Node{"r2d2", "robot"});
  graph_1.add_node(bica_graph::Node{"kitchen", "room"});
  
  ASSERT_TRUE(graph_1.add_edge(bica_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));

  std::cerr << graph_1.to_string() << std::endl;
  ASSERT_EQ(graph_1.get_num_nodes(), 2);
  ASSERT_EQ(graph_2.get_num_nodes(), 2);
  ASSERT_EQ(graph_2.get_num_edges(), 1);
}

TEST(bica_graph, typedgraph_node_operations)
{
  rclcpp::Node node("typedgraph_node_operations");

  bica_graph::TypedGraphNode graph_1("graph_test_node_1");
  bica_graph::TypedGraphNode graph_2("graph_test_node_2");

  ASSERT_FALSE(graph_1.exist_node("r2d2"));
  graph_1.add_node(bica_graph::Node{"r2d2", "robot"});
  graph_1.add_node(bica_graph::Node{"kitchen", "room"});
  
  //ASSERT_TRUE(
  graph_1.add_edge(bica_graph::Edge{"is", "symbolic", "r2d2", "kitchen"});

  ASSERT_EQ(graph_1.get_num_nodes(), 2);
  ASSERT_EQ(graph_2.get_num_nodes(), 2);
  ASSERT_EQ(graph_2.get_num_edges(), 1);

  tf2::Transform tf1(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 0, 0));
  bica_graph::TFEdge tf_edge_1;
  tf_edge_1.tf_.header.stamp = node.now();
  tf_edge_1.tf_.header.frame_id = "kitchen";
  tf_edge_1.tf_.child_frame_id = "r2d2";
  tf2::convert(tf1, tf_edge_1.tf_.transform);

  ASSERT_TRUE(graph_1.add_tf_edge(tf_edge_1));

  auto tf_edge_2 = graph_1.get_tf_edge("kitchen", "r2d2");
  ASSERT_TRUE(tf_edge_2.has_value());
  tf2::Stamped<tf2::Transform> tf2;
  tf2::convert(tf_edge_2.value().tf_, tf2);
  ASSERT_EQ(tf1, tf2);

  auto tf_edge_3 = graph_1.get_tf_edge("r2d2", "kitchen");
  ASSERT_TRUE(tf_edge_3.has_value());
  tf2::Stamped<tf2::Transform> tf3;
  tf2::convert(tf_edge_3.value().tf_, tf3);
  ASSERT_EQ(tf1, tf3.inverse());

  tf2::Transform tf4(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(1, 0, 0));
  bica_graph::TFEdge tf_edge_4;
  tf_edge_4.tf_.header.stamp = node.now();
  tf_edge_4.tf_.header.frame_id = "kitchen";
  tf_edge_4.tf_.child_frame_id = "r2d2";
  tf2::convert(tf4, tf_edge_4.tf_.transform);
  ASSERT_TRUE(graph_1.add_tf_edge(tf_edge_4));

  auto tf_edge_5 = graph_1.get_tf_edge("kitchen", "r2d2");
  ASSERT_TRUE(tf_edge_5.has_value());
  tf2::Stamped<tf2::Transform> tf5;
  tf2::convert(tf_edge_5.value().tf_, tf5);
  ASSERT_EQ(tf4, tf5);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}