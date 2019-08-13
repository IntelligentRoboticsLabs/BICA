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
#include <memory>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>

#include "bica_graph/graph.h"
#include "bica_graph/conversions.h"

TEST(BicaGraph, test_basic_elements)
{
  ros::Time::init();

  bica_graph::Node node_1("node_1", "type_1");
  ASSERT_EQ(node_1.get_id(), "node_1");
  ASSERT_EQ(node_1.get_type(), "type_1");

  bica_graph::Node node_2("node_2");
  ASSERT_EQ(node_2.get_id(), "node_2");
  ASSERT_EQ(node_2.get_type(), "no_type");

  bica_graph::Node node_3(node_1);
  ASSERT_EQ(node_3.get_id(), "node_1");
  ASSERT_EQ(node_3.get_type(), "type_1");

  ASSERT_EQ(node_1, node_3);
  ASSERT_NE(node_1, node_2);

  bica_graph::StringEdge str_edge_1("node_1", "is", "node_2");
  ASSERT_EQ(str_edge_1.get_source(), "node_1");
  ASSERT_EQ(str_edge_1.get_target(), "node_2");
  ASSERT_EQ(str_edge_1.get(), "is");

  bica_graph::StringEdge str_edge_2("node_1", "is", "node_2");
  ASSERT_EQ(str_edge_2.get_source(), "node_1");
  ASSERT_EQ(str_edge_2.get_target(), "node_2");
  ASSERT_EQ(str_edge_2.get(), "is");

  bica_graph::StringEdge str_edge_3("node_1", "not is", "node_2");

  ASSERT_EQ(str_edge_1, str_edge_2);
  ASSERT_NE(str_edge_1, str_edge_3);

  bica_graph::DoubleEdge dbl_edge_1("node_1", 0.5, "node_2");
  ASSERT_EQ(dbl_edge_1.get_source(), "node_1");
  ASSERT_EQ(dbl_edge_1.get_target(), "node_2");
  ASSERT_EQ(dbl_edge_1.get(), 0.5);

  bica_graph::DoubleEdge dbl_edge_2("node_1", 0.5, "node_2");
  ASSERT_EQ(dbl_edge_2.get_source(), "node_1");
  ASSERT_EQ(dbl_edge_2.get_target(), "node_2");
  ASSERT_EQ(dbl_edge_2.get(), 0.5);

  bica_graph::DoubleEdge dbl_edge_3("node_1", 0.8, "node_2");

  ASSERT_EQ(dbl_edge_1, dbl_edge_2);
  ASSERT_EQ(dbl_edge_1, dbl_edge_3);
  ASSERT_NE(dbl_edge_1.get(), dbl_edge_3.get());

  tf2::Transform tf1(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 0, 0));
  bica_graph::TFEdge tf_edge_1("node_1", tf1, "node_2");
  ASSERT_EQ(tf_edge_1.get_source(), "node_1");
  ASSERT_EQ(tf_edge_1.get_target(), "node_2");
  ASSERT_EQ(tf_edge_1.get().getOrigin(), tf2::Vector3(3, 0, 0));
  ASSERT_EQ(tf_edge_1.get(), tf1);

  bica_graph::TFEdge tf_edge_2(tf_edge_1);
  ASSERT_EQ(tf_edge_2.get_source(), "node_1");
  ASSERT_EQ(tf_edge_2.get_target(), "node_2");
  ASSERT_EQ(tf_edge_2.get(), tf1);

  bica_graph::TFEdge tf_edge_3("node_1", "node_2");
  ASSERT_EQ(tf_edge_3.get_source(), "node_1");
  ASSERT_EQ(tf_edge_3.get_target(), "node_2");
  ASSERT_EQ(tf_edge_3.get(), tf1);

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }
}
/*
TEST(BicaGraph, test_graph_construction)
{
  auto graph = std::make_shared<bica_graph::Graph>();

  graph->add_node("leia", "robot");
  ASSERT_TRUE(graph->exist_node("leia"));
  ASSERT_FALSE(graph->exist_node("paco"));

  try
  {
    graph->add_node("leia", "object");
    ASSERT_TRUE(false);
  }
  catch(bica_graph::exceptions::NodeTypeMismatch& e)
  {
    ASSERT_TRUE(true);
  }

  try
  {
    const bica_graph::Node& node_1 = graph->get_node("paco");
    ASSERT_TRUE(false);
  }
  catch(bica_graph::exceptions::NodeNotFound& e)
  {
    ASSERT_TRUE(true);
  }

  bica_graph::Node node_2 = graph->get_node("leia");
  ASSERT_EQ(node_2.get_id(), "leia");
  ASSERT_EQ(node_2.get_type(), "robot");

  ASSERT_EQ(graph->count_nodes(), 1);
  graph->remove_node("leia");
  ASSERT_EQ(graph->count_nodes(), 0);

  graph->add_node("house", "house");
  graph->add_node("leia", "robot");
  graph->add_node("bedroom", "robot");
  graph->add_edge("leia", "is", "bedroom");

  ASSERT_TRUE(graph->exist_node("bedroom"));
  ASSERT_TRUE(graph->exist_node("leia"));
  ASSERT_TRUE(graph->exist_edge("leia", "is", "bedroom"));
  ASSERT_EQ(graph->get_string_edges().size(), 1);

  graph->remove_node("leia");
  ASSERT_FALSE(graph->exist_edge("leia", "is", "bedroom"));
  ASSERT_EQ(graph->get_string_edges().size(), 0);

  tf2::Transform tf1(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 0, 0));
  tf2::Transform stf2(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(5, 0, 0));
  graph->add_node("leia", "robot");
  graph->add_edge("leia", "is", "bedroom");
  graph->add_edge("leia", "plays", "bedroom");
  graph->add_edge("leia", 0.5, "bedroom");
  graph->add_edge("bedroom", tf1, "leia");
  graph->add_edge("house", stf2, "bedroom", true);

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_TRUE(graph->exist_edge("leia", "is", "bedroom"));
  ASSERT_TRUE(graph->exist_edge("leia", "plays", "bedroom"));
  ASSERT_FALSE(graph->exist_edge("leia", "dreams", "bedroom"));
  ASSERT_TRUE(graph->exist_tf_edge("bedroom", "leia"));
  ASSERT_TRUE(graph->exist_tf_edge("house", "bedroom"));
  ASSERT_TRUE(graph->exist_double_edge("leia", "bedroom"));

  ASSERT_EQ(graph->get_string_edges().size(), 2);
  ASSERT_EQ(graph->get_double_edges().size(), 1);
  ASSERT_EQ(graph->get_tf_edges().size(), 2);

  ASSERT_EQ(graph->get_tf_edge("bedroom", "leia").get(), tf1);
  ASSERT_EQ(graph->get_tf_edge("house", "bedroom").get(), stf2);

  graph->get_double_edge("leia", "bedroom").set(0.9);
  ASSERT_EQ(graph->get_double_edge("leia", "bedroom").get(), 0.9);

  tf2::Transform tf2(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(2, 0, 0));
  graph->get_tf_edge("bedroom", "leia").set(tf2);

  ASSERT_FALSE(graph->get_tf_edge("bedroom", "leia").is_static());
  ASSERT_TRUE(graph->get_tf_edge("house", "bedroom").is_static());

  c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  graph->print();
  ASSERT_EQ(graph->get_tf_edge("bedroom", "leia").get(), tf2);

  graph->remove_edge("leia", "is", "bedroom");
  graph->remove_edge("leia", "plays", "bedroom");
  graph->remove_tf_edge("bedroom", "leia");
  graph->remove_tf_edge("house", "bedroom");
  graph->remove_double_edge("leia", "bedroom");

  ASSERT_EQ(graph->get_string_edges().size(), 0);
  ASSERT_EQ(graph->get_double_edges().size(), 0);
  ASSERT_EQ(graph->get_tf_edges().size(), 0);
}


TEST(BicaGraph, basic_types_conversions)
{
  ros::Time::init();
  ros::NodeHandle nh;

  bica_graph::StringEdge edge_1("node_source", "edge_1", "node_target");
  bica_msgs::Edge edge_1_msg;

  bica_graph::edge_to_msg(edge_1, &edge_1_msg);
  ASSERT_EQ(edge_1_msg.type, bica_msgs::Edge::EDGE_TYPE_STRING);
  ASSERT_EQ(edge_1_msg.string_data, "edge_1");
  ASSERT_EQ(edge_1_msg.source, "node_source");
  ASSERT_EQ(edge_1_msg.target, "node_target");

  bica_graph::DoubleEdge edge_2("node_source", 0.5, "node_target");
  bica_msgs::Edge edge_2_msg;

  bica_graph::edge_to_msg(edge_2, &edge_2_msg);
  ASSERT_EQ(edge_2_msg.type, bica_msgs::Edge::EDGE_TYPE_DOUBLE);
  ASSERT_EQ(edge_2_msg.double_data, 0.5);

  tf2::Transform tf(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 0, 0));
  bica_graph::TFEdge edge_3("node_source", tf, "node_target");
  bica_msgs::Edge edge_3_msg;

  bica_graph::edge_to_msg(edge_3, &edge_3_msg);
  ASSERT_EQ(edge_3_msg.type, bica_msgs::Edge::EDGE_TYPE_TF);
}

TEST(BicaGraph, graph_conversions)
{
  auto graph = std::make_shared<bica_graph::Graph>();
  graph->add_node("leia", "robot");
  graph->add_node("apple", "object");
  graph->add_node("bedroom", "room");

  graph->add_edge("leia", std::string("is"), "bedroom");
  graph->add_edge("leia", std::string("is"), "bedroom");
  graph->add_edge("leia", "isbis", "bedroom");
  graph->add_edge("bedroom", tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(2, 0, 0)), "leia");
  graph->add_edge("bedroom", 0.5, "leia");

  bica_msgs::Graph msg;
  bica_graph::graph_to_msg(*graph, &msg);

  bica_graph::Graph::SharedPtr graph_2;
  bica_graph::msg_to_graph(msg, graph_2);

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(*graph, *graph_2);
}
*/

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "graph_tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
