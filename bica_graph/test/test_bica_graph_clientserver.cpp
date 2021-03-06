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

#include <tf2_ros/transform_listener.h>

#include <string>

#include <bica_graph/graph_client.h>
#include <bica_graph/graph_server.h>

class GraphServerTest: public bica_graph::GraphServer
{
public:
  bica_graph::Graph::ConstSharedPtr get_graph() {return graph_;}
};

class GraphClientTest: public bica_graph::GraphClient
{
public:
  bica_graph::Graph::ConstSharedPtr get_graph() {return graph_;}
};


TEST(BicaGraph, test_graph_construction)
{
  ros::Time::init();
  ros::AsyncSpinner spinner(2);
  spinner.start();

  auto server = std::make_shared<GraphServerTest>();
  auto client_1 = std::make_shared<GraphClientTest>();
  auto client_2 = std::make_shared<GraphClientTest>();

  try { client_1->add_node("leia", "robot"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  ros::Time start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  ASSERT_EQ(1, client_1->count_nodes());
  ASSERT_EQ(1, client_2->count_nodes());

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  try { client_2->remove_node("leia"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  ASSERT_EQ(0, client_1->count_nodes());
  ASSERT_EQ(0, client_2->count_nodes());

  try { client_1->add_node("leia", "robot"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->add_node("bedroom", "room"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->add_edge("leia", "is", "bedroom"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 1.5 ) {}

  ASSERT_TRUE(client_2->exist_edge("leia", "is", "bedroom"));

  tf2::Transform tf_r2l(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 0, 0));
  client_1->add_edge("bedroom", tf_r2l, "leia");

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  ASSERT_TRUE(client_2->exist_tf_edge("bedroom", "leia"));
  try
  {
    auto edge_2 = client_2->get_tf_edge("bedroom", "leia");
    ASSERT_EQ(tf_r2l, edge_2.get());
    ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  try
  {
    client_1->add_edge("bedroom", 0.95, "leia");
    ASSERT_TRUE(true);
  }
  catch(std::exception* e)
  {
    ROS_ERROR("%s", e->what());
    ASSERT_TRUE(false);
  }

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  ASSERT_TRUE(client_2->exist_double_edge("bedroom", "leia"));

  try
  {
    auto edge_3 = client_2->get_double_edge("bedroom", "leia");
    ASSERT_EQ(0.95, edge_3.get());
    ASSERT_TRUE(true);
  }
  catch(...)
  {
    ASSERT_TRUE(false);
  }

  try { client_1->remove_double_edge("bedroom", "leia"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->remove_tf_edge("bedroom", "leia"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->remove_edge("leia", "is", "bedroom"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  spinner.stop();
}

TEST(BicaGraph, test_graph_tf)
{
  ros::Time::init();
  ros::AsyncSpinner spinner(2);
  spinner.start();
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_listener(tfBuffer);

  auto server = std::make_shared<GraphServerTest>();
  auto client_1 = std::make_shared<GraphClientTest>();
  auto client_2 = std::make_shared<GraphClientTest>();

  try { client_1->add_node("leia", "robot"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->add_node("world", "abstract"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->add_node("ball", "object"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  ros::Time start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 2.0 ) {}

  try { client_1->set_tf_identity("base_footprint", "leia"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->set_tf_identity("odom", "world"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }

  tf2::Transform tf_r2b(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(3, 0, 0));
  tf2::Transform tf_w2r(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(1, 0, 0));

  try { client_1->add_edge("world", tf_w2r, "leia"); ASSERT_TRUE(true); } catch(...) { ASSERT_TRUE(false); }
  try { client_1->add_edge("leia", tf_r2b, "ball"); ASSERT_TRUE(true);}
  catch(std::exception* e)
  {
    ROS_ERROR("%s", e->what());
    ASSERT_TRUE(false);
  }

  start = ros::Time::now();
  while ((ros::Time::now() - start).toSec() < 3.0 ) {}

  try
  {
    tf2::Vector3 v1 = client_1->get_tf("world", "ball").getOrigin();
    ASSERT_EQ(tf2::Vector3(4, 0, 0), v1);
  }
  catch(bica_graph::exceptions::TransformNotPossible& e)
  {
    std::cerr << e.what() << std::endl;
    ASSERT_TRUE(false);
  }

  geometry_msgs::TransformStamped tf;
  std::string error;
  if (tfBuffer.canTransform("odom", "ball", ros::Time(0), ros::Duration(0.1), &error))
  {
    tf2::Stamped<tf2::Transform> tf2;
    tf = tfBuffer.lookupTransform("odom", "ball", ros::Time(0));
    tf2::fromMsg(tf, tf2);
    ASSERT_EQ(tf2::Vector3(4, 0, 0), tf2.getOrigin());
  }
  else
  {
    ROS_ERROR("Can't transform %s", error.c_str());
    ASSERT_TRUE(false);
  }

  spinner.stop();
}

TEST(BicaGraph, test_advanced_funcs)
{
  /*
  ros::Time::init();
  ros::AsyncSpinner spinner(2);
  spinner.start();

  auto server = std::make_shared<GraphServerTest>();
  auto client = std::make_shared<GraphClientTest>();

  client->add_node("leia", "robot");
  client->add_node("world", "abstract");
  client->add_node("ball", "object");
  client->add_node("table_1", "table");
  client->add_node("table_2", "table");
  client->add_node("table_3", "table");
  client->add_node("table_4", "table");

  ASSERT_EQ(client->get_node_names_by_id("[[:alnum:]_]*").size(), 7);
  ASSERT_EQ(client->get_node_names_by_id("table_[[:alnum:]_]*").size(), 4);
  ASSERT_EQ(client->get_node_names_by_id("table_1").size(), 1);
  ASSERT_EQ(client->get_node_names_by_id("kajshd").size(), 0);


  ASSERT_EQ(client->get_node_names_by_id("table_[[:alnum:]_]*")[0], "table_1");
  ASSERT_EQ(client->get_node_names_by_id("table_[[:alnum:]_]*")[1], "table_2");
  ASSERT_EQ(client->get_node_names_by_id("table_[[:alnum:]_]*")[2], "table_3");
  ASSERT_EQ(client->get_node_names_by_id("table_[[:alnum:]_]*")[3], "table_4");


  ASSERT_EQ(client->get_node_names_by_type("table").size(), 4);
  ASSERT_EQ(client->get_node_names_by_type("ball").size(), 0);
  ASSERT_EQ(client->get_node_names_by_type("robot").size(), 1);

  ASSERT_EQ(client->get_node_names_by_type("table")[0], "table_1");
  ASSERT_EQ(client->get_node_names_by_type("table")[1], "table_2");
  ASSERT_EQ(client->get_node_names_by_type("table")[2], "table_3");
  ASSERT_EQ(client->get_node_names_by_type("table")[3], "table_4");

  client->add_edge("leia", "is_near", "table_1");
  client->add_edge("leia", "is_near", "table_2");
  client->add_edge("leia", "is_far", "table_3");
  client->add_edge("leia", "not_sees", "table_4");
  client->add_edge("world", "contains_robot", "leia");
  client->add_edge("ball", "is_near", "leia");

  ASSERT_EQ(client->get_string_edges_from_node("leia").size(), 4);
  ASSERT_EQ(client->get_string_edges_from_node("world").size(), 1);
  ASSERT_EQ(client->get_string_edges_from_node("table_1").size(), 0);
  ASSERT_EQ(client->get_string_edges_from_node("kjashdfkjhsd").size(), 0);


  ASSERT_EQ(client->get_string_edges_from_node("leia")[0].get(), "is_near");
  ASSERT_EQ(client->get_string_edges_from_node("leia")[0].get_source(), "leia");
  ASSERT_EQ(client->get_string_edges_from_node("leia")[0].get_target(), "table_1");
  ASSERT_EQ(client->get_string_edges_from_node("leia")[1].get(), "is_near");
  ASSERT_EQ(client->get_string_edges_from_node("leia")[2].get(), "is_far");

  ASSERT_EQ(client->get_string_edges_from_node_by_data("leia", "is_[[:alnum:]_]*").size(), 3);
  ASSERT_EQ(client->get_string_edges_from_node_by_data("leia", "is_near").size(), 2);
  ASSERT_EQ(client->get_string_edges_from_node_by_data("leia", "is_far").size(), 1);

  ASSERT_EQ(client->get_string_edges_from_node_by_data("leia",
    "is_[[:alnum:]_]*")[0].get_target(), "table_1");
  ASSERT_EQ(client->get_string_edges_from_node_by_data("leia",
    "is_[[:alnum:]_]*")[1].get_target(), "table_2");

  ASSERT_EQ(client->get_string_edges_by_data("is_[[:alnum:]_]*").size(), 4);
  ASSERT_EQ(client->get_string_edges_by_data("is_near").size(), 3);

  ASSERT_EQ(client->get_string_edges_by_data("is_near")[0].get_source(), "leia");
  ASSERT_EQ(client->get_string_edges_by_data("is_near")[0].get_target(), "table_1");

  ASSERT_EQ(client->get_string_edges_by_data("is_near")[2].get_source(), "ball");
  ASSERT_EQ(client->get_string_edges_by_data("is_near")[2].get_target(), "leia");*/
}


int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "graph_tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
