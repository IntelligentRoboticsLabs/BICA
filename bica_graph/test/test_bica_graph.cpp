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

#include <geometry_msgs/TransformStamped.h>

#include <bica_graph/graph.h>
#include <bica_graph/node.h>
#include <bica_graph/relation.h>
#include <bica_graph/tf_relation.h>
#include <bica_graph/exceptions.h>
#include <bica_graph/conversions.h>
#include <bica_msgs/Graph.h>
#include <bica_graph/graph_handler.h>

class GraphHandlerTest
{
public:
  explicit GraphHandlerTest(std::string id): nh_(), graph_handler_(nh_, id) {}

  ros::NodeHandle nh_;
  bica_graph::GraphHandler graph_handler_;
};

TEST(BicaGraph, test_graph_construction)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  ASSERT_FALSE(graph->is_sub_graph());
  ASSERT_EQ(0, graph->count_nodes());
}

TEST(BicaGraph, test_node_and_relation)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  auto node = graph->create_node("leia", "robot");
  auto bedroom =  graph->create_node("bedroom", "room");
  auto relation1 = node->add_relation("is", bedroom);
  auto relation2 = node->add_relation("isbis", bedroom);

  ASSERT_EQ(2, graph->count_nodes());
  ASSERT_EQ(2, node->get_relations().size());
  ASSERT_EQ(2, bedroom->get_incoming_relations().size());
  ASSERT_EQ("leia", node->get_id());
  ASSERT_EQ("is", relation1->get_type());
  ASSERT_EQ("is", node->get_relations().front()->get_type());
  ASSERT_EQ("is", bedroom->get_incoming_relations().front()->get_type());
  ASSERT_EQ("leia", node->get_relations().front()->get_source()->get_id());
  ASSERT_EQ("leia", bedroom->get_incoming_relations().front()->get_source()->get_id());
  ASSERT_EQ("bedroom", node->get_relations().front()->get_target()->get_id());
  ASSERT_EQ("bedroom", bedroom->get_incoming_relations().front()->get_target()->get_id());

  node->remove_relation("isbis", bedroom);
  ASSERT_EQ(1, node->get_relations().size());

  graph->remove_node("bedroom");
  ASSERT_EQ(1, graph->count_nodes());
  ASSERT_EQ(0, node->get_relations().size());
}

TEST(BicaGraph, test_node_and_tf_relation)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  auto node = graph->create_node("leia", "robot");
  auto bedroom =  graph->create_node("bedroom", "room");

  tf::Transform tf(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));

  auto relation1 = bedroom->add_tf_relation(tf, node);

  ASSERT_EQ(2, graph->count_nodes());
  ASSERT_EQ(1, bedroom->get_relations().size());
  ASSERT_EQ("leia", node->get_id());
  ASSERT_EQ("tf", relation1->get_type());
  ASSERT_EQ("tf", bedroom->get_relations().front()->get_type());
  ASSERT_EQ("bedroom", bedroom->get_relations().front()->get_source()->get_id());
  ASSERT_EQ("leia", bedroom->get_relations().front()->get_target()->get_id());
  ASSERT_EQ("bedroom", relation1->get_transform().frame_id_);
  ASSERT_EQ("leia", relation1->get_transform().child_frame_id_);
  ASSERT_EQ(tf, relation1->get_transform());

  auto r =  std::dynamic_pointer_cast<bica_graph::TFRelation>(bedroom->get_relations().front());

  ASSERT_EQ("bedroom", r->get_transform().frame_id_);
  ASSERT_EQ("leia", r->get_transform().child_frame_id_);
  ASSERT_EQ(tf, r->get_transform());
}

TEST(BicaGraph, test_conversion_to_msg)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  auto node = graph->create_node("leia", "robot");
  auto bedroom =  graph->create_node("bedroom", "room");
  auto relation1 = node->add_relation("is", bedroom);
  auto relation2 = node->add_relation("isbis", bedroom);

  tf::Transform tf(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));

  auto relation_tf = node->add_tf_relation(tf, bedroom);

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 20)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  bica_msgs::Graph::ConstPtr msg = graph_to_msg(*graph);

  ASSERT_EQ(2, msg->nodes.size());

  for (int i = 0; i < msg->nodes.size(); i++)
  {
    if (i == 0)
    {
      ASSERT_EQ("leia", msg->nodes[i].id);
      ASSERT_EQ("robot", msg->nodes[i].type);

      ASSERT_EQ(2, msg->nodes[i].relations.size());
      ASSERT_EQ(1, msg->nodes[i].tf_relations.size());
    }

    if (i == 1)
    {
      ASSERT_EQ("bedroom", msg->nodes[i].id);
      ASSERT_EQ("room", msg->nodes[i].type);

      ASSERT_EQ(0, msg->nodes[i].relations.size());
      ASSERT_EQ(0, msg->nodes[i].tf_relations.size());
    }
  }
}

TEST(BicaGraph, test_conversion_from_msg)
{
  bica_msgs::Graph::Ptr msg(new bica_msgs::Graph());

  bica_msgs::Node node;
  node.id = "leia";
  node.type = "robot";
  bica_msgs::Node bedroom;
  bedroom.id = "bedroom";
  bedroom.type = "room";

  bica_msgs::Relation relation1;
  relation1.source = "leia";
  relation1.type = "is";
  relation1.target = "bedroom";
  bica_msgs::Relation relation2;
  relation2.source = "leia";
  relation2.type = "isbis";
  relation2.target = "bedroom";

  bica_msgs::TFRelation relation_tf;
  relation_tf.source = "leia";
  relation_tf.target = "bedroom";
  tf::Transform tf(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));
  tf::transformTFToMsg(tf, relation_tf.transform);

  node.relations.push_back(relation1);
  node.relations.push_back(relation2);
  node.tf_relations.push_back(relation_tf);

  msg->nodes.push_back(node);
  msg->nodes.push_back(bedroom);

  auto graph_msg = bica_graph::msg_to_graph(msg);

  auto graph = std::make_shared<bica_graph::BicaGraph>();

  auto g_node = graph->create_node("leia", "robot");
  auto g_bedroom =  graph->create_node("bedroom", "room");
  auto g_relation1 = g_node->add_relation("is", g_bedroom);
  auto g_relation2 = g_node->add_relation("isbis", g_bedroom);

  tf::Transform tf2(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 1));

  auto g_relation_tf = g_node->add_tf_relation(tf2, g_bedroom);

  ASSERT_EQ(*graph_msg, *graph);
}

TEST(BicaGraph, test_timestamps)
{
  ros::Time::init();
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  ros::Time t1 = ros::Time::now();

  auto node = graph->create_node("leia", "robot", t1);
  auto bedroom =  graph->create_node("bedroom", "room", t1);
  auto relation1 = node->add_relation("is", bedroom, t1);
  auto relation2 = node->add_relation("isbis", bedroom, t1);

  ros::Time ts_node = graph->get_node("leia")->get_time_stamp();
  ASSERT_EQ(ts_node, t1);
  ros::Time ts_bedroom = graph->get_node("bedroom")->get_time_stamp();
  ASSERT_EQ(ts_bedroom, t1);
  ros::Time ts_r1 = graph->get_node("leia")->get_relation(bedroom, "is")->get_time_stamp();
  ASSERT_EQ(ts_r1, t1);
  ros::Time ts_r2 = graph->get_node("leia")->get_relation(bedroom, "isbis")->get_time_stamp();
  ASSERT_EQ(ts_r2, t1);

  graph->get_node("leia")->update_time_stamp();
  graph->create_node("bedroom", "room");
  graph->get_node("leia")->get_relation(bedroom, "is")->update_time_stamp();
  graph->get_node("leia")->get_relation(bedroom, "isbis")->update_time_stamp();

  ros::Time ts_node_2 = graph->get_node("leia")->get_time_stamp();
  ASSERT_GT((ts_node_2 - t1).toSec(), 0);
  ASSERT_LT((ts_node_2 - t1).toSec(), 0.1);
  ros::Time ts_bedroom_2 = graph->get_node("bedroom")->get_time_stamp();
  ASSERT_GT((ts_bedroom_2 - t1).toSec(), 0);
  ASSERT_LT((ts_bedroom_2 - t1).toSec(), 0.1);
  ros::Time ts_r1_2 = graph->get_node("leia")->get_relation(bedroom, "is")->get_time_stamp();
  ASSERT_GT((ts_r1_2 - t1).toSec(), 0);
  ASSERT_LT((ts_r1_2 - t1).toSec(), 0.1);
  ros::Time ts_r2_2 = graph->get_node("leia")->get_relation(bedroom, "isbis")->get_time_stamp();
  ASSERT_GT((ts_r2_2 - t1).toSec(), 0);
  ASSERT_LT((ts_r2_2 - t1).toSec(), 0.1);
}

TEST(BicaGraph, test_handler_node_api)
{
  GraphHandlerTest graph_handler_1("handler_1");
  GraphHandlerTest graph_handler_2("handler_2");

  graph_handler_1.graph_handler_.create_node("leia", "robot");
  graph_handler_1.graph_handler_.create_node("bedroom", "room");

  ASSERT_EQ(2, graph_handler_1.graph_handler_.count_nodes());
  ASSERT_EQ(0, graph_handler_2.graph_handler_.count_nodes());

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(2, graph_handler_2.graph_handler_.count_nodes());
  auto node_1 = graph_handler_2.graph_handler_.get_node("leia");
  ASSERT_NE(nullptr, node_1);
  ASSERT_EQ("leia", node_1->get_id());
  ASSERT_EQ("robot", node_1->get_type());
  auto node_2 = graph_handler_2.graph_handler_.get_node("bedroom");
  ASSERT_NE(nullptr, node_2);
  ASSERT_EQ("bedroom", node_2->get_id());
  ASSERT_EQ("room", node_2->get_type());

  graph_handler_2.graph_handler_.create_node("apple", "object");

  c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(3, graph_handler_1.graph_handler_.count_nodes());
  ASSERT_EQ(3, graph_handler_2.graph_handler_.count_nodes());
  auto node_3 = graph_handler_1.graph_handler_.get_node("apple");
  ASSERT_NE(nullptr, node_3);
  ASSERT_EQ("apple", node_3->get_id());
  ASSERT_EQ("object", node_3->get_type());

  graph_handler_2.graph_handler_.remove_node("apple");

  c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(2, graph_handler_1.graph_handler_.count_nodes());
  ASSERT_EQ(2, graph_handler_2.graph_handler_.count_nodes());
  auto node_4 = graph_handler_1.graph_handler_.get_node("apple");
  ASSERT_EQ(nullptr, node_4);


  graph_handler_1.graph_handler_.remove_node("leia");
  graph_handler_2.graph_handler_.create_node("leia", "robot");

  c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(2, graph_handler_1.graph_handler_.count_nodes());
  ASSERT_EQ(2, graph_handler_2.graph_handler_.count_nodes());

  auto node_5 = graph_handler_1.graph_handler_.get_node("leia");
  auto node_6 = graph_handler_2.graph_handler_.get_node("leia");
  ASSERT_NE(nullptr, node_5);
  ASSERT_NE(nullptr, node_6);


  /* This is a concurrency problem that can arise, but not easy to fix

  graph_handler_1.graph_handler_.create_node("leia", "robot");
  graph_handler_2.graph_handler_.remove_node("leia");

  c = 0;
  while(ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(1, graph_handler_1.graph_handler_.count_nodes());
  ASSERT_EQ(1, graph_handler_2.graph_handler_.count_nodes());

  auto node_7 = graph_handler_1.graph_handler_.get_node("leia");
  auto node_8 = graph_handler_2.graph_handler_.get_node("leia");
  ASSERT_EQ(nullptr, node_7);
  ASSERT_EQ(nullptr, node_8);
  */

  // graph_handler_1.graph_handler_.create_relation("leia", "is", "bedroom");


  /* graph_handler->add_relation("leia", "is", "bedroom");
  graph_handler->add_relation("leia", "isbis", "bedroom");

  geometry_msgs::TransformStamped tf;
  tf.header.seq = 1;
  tf.header.frame_id = "/world";
  tf.child_frame_id = "/robot";
  tf.translation.x = 1;
  tf.translation.y = 1;
  tf.translation.z = 1;
  tf.rotation.x = 0;
  tf.rotation.y = 0;
  tf.rotation.z = 0;
  tf.rotation.w = 1;

  graph_handler->add_tf_relation("leia", ft, "bedroom");

  auto node_leia = graph_handler->get_node("leia");
  graph_handler->remove_node("leia");
  */
}


TEST(BicaGraph, test_handler_relation_api)
{
  GraphHandlerTest graph_handler_1("handler_3");
  GraphHandlerTest graph_handler_2("handler_4");

  graph_handler_1.graph_handler_.create_node("leia", "robot");
  graph_handler_1.graph_handler_.create_node("bedroom", "room");

  ASSERT_EQ(2, graph_handler_1.graph_handler_.count_nodes());
  ASSERT_EQ(0, graph_handler_2.graph_handler_.count_nodes());

  ros::Rate rate(20);
  int c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  graph_handler_1.graph_handler_.add_relation("leia", "is", "bedroom");
  graph_handler_1.graph_handler_.add_relation("leia", "isbis", "bedroom");

  auto node_1 = graph_handler_1.graph_handler_.get_node("leia");
  auto node_2 = graph_handler_2.graph_handler_.get_node("leia");

  ASSERT_EQ(2, node_1->count_relations());
  ASSERT_EQ(0, node_2->count_relations());

  c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(2, node_1->count_relations());
  ASSERT_EQ(2, node_2->count_relations());

  graph_handler_1.graph_handler_.remove_relation("leia", "is", "bedroom");

  c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(1, node_1->count_relations());
  ASSERT_EQ(1, node_2->count_relations());

  graph_handler_1.graph_handler_.remove_node("bedroom");

  c = 0;
  while (ros::ok() && c < 10)
  {
    c++;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(0, node_1->count_relations());
  ASSERT_EQ(0, node_2->count_relations());
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "graph_tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
