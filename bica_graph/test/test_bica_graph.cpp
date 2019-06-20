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

#include <bica_graph/graph.h>

TEST(BicaGraph, test_graph_construction)
{
  auto graph = std::make_shared<bica_graph::Graph>();

  graph->add_node("leia", "robot");

  try
  {
    graph->add_node("leia", "object");
    ASSERT_TRUE(false);
  }
  catch(bica_graph::exceptions::NodeTypeMismatch& e)
  {
    // printf("Exception: %s\n", e.what());
    ASSERT_TRUE(true);
  }

  try
  {
    graph->get_node("apple");
    ASSERT_TRUE(false);
  }
  catch(bica_graph::exceptions::NodeNotFound& e)
  {
    // printf("Exception: %s\n", e.what());
    ASSERT_TRUE(true);
  }

  graph->add_node("apple", "object");

  auto node = graph->get_node("leia");
  ASSERT_EQ("robot", node->get_type());

  graph->add_node("bedroom", "room");

  graph->add_edge("leia", std::string("is"), "bedroom");
  graph->add_edge("leia", std::string("is"), "bedroom");
  graph->add_edge<std::string>("leia", "isbis", "bedroom");

  tf::Transform tf_b2l(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 0, 0));
  graph->add_edge("bedroom", tf_b2l, "leia");

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  tf::Transform tf_b2l_bis(tf::Quaternion(0, 0, 0, 1), tf::Vector3(3, 0, 0));
  graph->add_edge("bedroom", tf_b2l_bis, "leia");

  c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  graph->add_edge("bedroom", 0.5, "leia");

  ASSERT_TRUE(graph->exist_edge<std::string>("leia", "bedroom", "is"));
  ASSERT_TRUE(graph->exist_edge<std::string>("leia", "bedroom", "isbis"));
  ASSERT_FALSE(graph->exist_edge<std::string>("leia", "bedroom", "destroy"));

  auto edge_1 = graph->get_edge<double>("bedroom", "leia");

  ASSERT_EQ(0.5, edge_1->get());

  ASSERT_EQ(3, graph->count_nodes());
  ASSERT_EQ(2, graph->count_edges("leia", "bedroom"));
  ASSERT_EQ(2, graph->count_edges("bedroom", "leia"));
  ASSERT_EQ(0, graph->count_edges("apple", "leia"));

  auto edge_2 =  std::dynamic_pointer_cast<bica_graph::Edge<tf::Transform>>(
      graph->get_edge<tf::Transform>("bedroom", "leia"));

  tf::Transform tf_ = edge_2->get();
  ASSERT_EQ(tf_b2l_bis, edge_2->get());
  ASSERT_FALSE(tf_b2l == edge_2->get());

  graph->remove_edge<std::string>("leia", "bedroom", "is");
  graph->remove_edge<tf::Transform>("bedroom", "leia");

  ASSERT_FALSE(graph->exist_edge<std::string>("leia", "bedroom", "is"));
  ASSERT_FALSE(graph->exist_edge<tf::Transform>("bedroom", "leia"));
  ASSERT_EQ(1, graph->count_edges("leia", "bedroom"));
  ASSERT_EQ(1, graph->count_edges("bedroom", "leia"));

  graph->remove_node("bedroom");
  ASSERT_EQ(2, graph->count_nodes());
  ASSERT_EQ(0, graph->count_edges("leia", "bedroom"));
  ASSERT_EQ(0, graph->count_edges("bedroom", "leia"));
  ASSERT_EQ(0, graph->count_edges("apple", "leia"));

  ros::Time::init();

  auto node_1 = std::make_shared<bica_graph::Node>("car", "vehicle");
  graph->add_node(*node_1);
  auto node_1_g = graph->get_node("car");
  ASSERT_EQ(*node_1, *node_1_g);

  graph->add_node("cycle", "vehicle");
  auto edge_x = std::make_shared<bica_graph::Edge<std::string>>("car", "cycle", "near");
  graph->add_edge(*edge_x);
  auto edge_x_g = graph->get_edge<std::string>("car", "cycle", "near");
  ASSERT_EQ(*edge_x, *edge_x_g);
}

TEST(BicaGraph, test_timestamps)
{
}

TEST(BicaGraph, basic_types_conversions)
{
  ros::Time::init();
  ros::NodeHandle nh;

  auto node_source = std::make_shared<bica_graph::Node>("leia", "robot");
  bica_msgs::Node node_source_msg;
  bica_graph::node_to_msg(node_source, &node_source_msg);

  ASSERT_EQ(node_source_msg.type, "robot");

  auto edge_1 = std::make_shared<bica_graph::Edge<std::string>>("node_source", "node_target", "edge_1");
  bica_msgs::Edge edge_1_msg;

  bica_graph::edge_to_msg<std::string>(edge_1, &edge_1_msg);
  ASSERT_EQ(edge_1_msg.type, bica_msgs::Edge::EDGE_TYPE_STRING);
  ASSERT_EQ(edge_1_msg.string_data, "edge_1");
  ASSERT_EQ(edge_1_msg.source, "node_source");
  ASSERT_EQ(edge_1_msg.target, "node_target");

  auto edge_2 = std::make_shared<bica_graph::Edge<double>>("node_source", "node_target", 0.5);
  bica_msgs::Edge edge_2_msg;

  bica_graph::edge_to_msg<double>(edge_2, &edge_2_msg);
  ASSERT_EQ(edge_2_msg.type, bica_msgs::Edge::EDGE_TYPE_DOUBLE);
  ASSERT_EQ(edge_2_msg.double_data, 0.5);


  tf::Transform tf(tf::Quaternion(0, 0, 0, 1), tf::Vector3(3, 0, 0));
  auto edge_3 = std::make_shared<bica_graph::Edge<tf::Transform>>("node_source", "node_target", tf);
  bica_msgs::Edge edge_3_msg;

  bica_graph::edge_to_msg<tf::Transform>(edge_3, &edge_3_msg);
  ASSERT_EQ(edge_3_msg.type, bica_msgs::Edge::EDGE_TYPE_TF);


  bica_graph::Node::SharedPtr node_source_2;
  bica_graph::msg_to_node(node_source_msg, node_source_2);
  ASSERT_EQ(*node_source_2, *node_source);


  std::shared_ptr<bica_graph::Edge<std::string>> edge_1_2;
  bica_graph::msg_to_edge(edge_1_msg, edge_1_2);

  ASSERT_EQ(edge_1_2->get_source(), "node_source");
  ASSERT_EQ(edge_1_2->get_target(), "node_target");
  ASSERT_EQ(*edge_1, *edge_1_2);

  std::shared_ptr<bica_graph::Edge<double>> edge_2_2;
  bica_graph::msg_to_edge(edge_2_msg, edge_2_2);
  ASSERT_EQ(*edge_2, *edge_2_2);

  std::shared_ptr<bica_graph::Edge<tf::Transform>> edge_3_2;
  bica_graph::msg_to_edge(edge_3_msg, edge_3_2);
  ASSERT_EQ(*edge_3, *edge_3_2);
}

TEST(BicaGraph, graph_conversions)
{
  auto graph = std::make_shared<bica_graph::Graph>();
  graph->add_node("leia", "robot");
  graph->add_node("apple", "object");
  graph->add_node("bedroom", "room");

  graph->add_edge("leia", std::string("is"), "bedroom");
  graph->add_edge("leia", std::string("is"), "bedroom");
  graph->add_edge<std::string>("leia", "isbis", "bedroom");
  graph->add_edge("bedroom", tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 0, 0)), "leia");
  graph->add_edge("bedroom", 0.5, "leia");

  bica_msgs::Graph msg;
  bica_graph::graph_to_msg(*graph, &msg);

  bica_graph::Graph::SharedPtr graph_2;
  bica_graph::msg_to_graph(msg, graph_2);

  ASSERT_EQ(*graph, *graph_2);
}



int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "graph_tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
