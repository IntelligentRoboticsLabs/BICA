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

#include <geometry_msgs/TransformStamped.h>

#include <bica_graph/graph.h>
#include <bica_graph/node.h>
#include <bica_graph/relation.h>
#include <bica_graph/tf_relation.h>
#include <bica_graph/exceptions.h>
#include <bica_graph/conversions.h>
#include <bica_msgs/Graph.h>

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
  ASSERT_EQ("leia", node->get_id());
  ASSERT_EQ("is", relation1->get_type());
  ASSERT_EQ("is", node->get_relations().front()->get_type());
  ASSERT_EQ("leia", node->get_relations().front()->get_source()->get_id());
  ASSERT_EQ("bedroom", node->get_relations().front()->get_target()->get_id());
}

TEST(BicaGraph, test_node_and_tf_relation)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  auto node = graph->create_node("leia", "robot");
  auto bedroom =  graph->create_node("bedroom", "room");

  geometry_msgs::TransformStamped tf;
  tf.header.seq = 1;
  tf.header.frame_id = "/world";
  tf.child_frame_id = "/robot";
  tf.transform.translation.x = 1;
  tf.transform.translation.y = 1;
  tf.transform.translation.z = 1;
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;

  auto relation1 = node->add_tf_relation(tf, bedroom);

  ASSERT_EQ(2, graph->count_nodes());
  ASSERT_EQ(1, node->get_relations().size());
  ASSERT_EQ("leia", node->get_id());
  ASSERT_EQ("tf", relation1->get_type());
  ASSERT_EQ("tf", node->get_relations().front()->get_type());
  ASSERT_EQ("leia", node->get_relations().front()->get_source()->get_id());
  ASSERT_EQ("bedroom", node->get_relations().front()->get_target()->get_id());
  ASSERT_EQ(1, relation1->get_transform().header.seq);
  ASSERT_EQ("/world", relation1->get_transform().header.frame_id);
  ASSERT_EQ("/robot", relation1->get_transform().child_frame_id);
  ASSERT_EQ(1, relation1->get_transform().transform.translation.x);
  ASSERT_EQ(1, relation1->get_transform().transform.translation.y);
  ASSERT_EQ(1, relation1->get_transform().transform.translation.z);
  ASSERT_EQ(0, relation1->get_transform().transform.rotation.x);
  ASSERT_EQ(0, relation1->get_transform().transform.rotation.y);
  ASSERT_EQ(0, relation1->get_transform().transform.rotation.z);
  ASSERT_EQ(1, relation1->get_transform().transform.rotation.w);

  auto r =  std::dynamic_pointer_cast<bica_graph::TFRelation>(node->get_relations().front());

  ASSERT_EQ(1, r->get_transform().header.seq);
  ASSERT_EQ("/world", r->get_transform().header.frame_id);
  ASSERT_EQ("/robot", r->get_transform().child_frame_id);
  ASSERT_EQ(1, r->get_transform().transform.translation.x);
  ASSERT_EQ(1, r->get_transform().transform.translation.y);
  ASSERT_EQ(1, r->get_transform().transform.translation.z);
  ASSERT_EQ(0, r->get_transform().transform.rotation.x);
  ASSERT_EQ(0, r->get_transform().transform.rotation.y);
  ASSERT_EQ(0, r->get_transform().transform.rotation.z);
  ASSERT_EQ(1, r->get_transform().transform.rotation.w);
}

TEST(BicaGraph, test_conversion_to_msg)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  auto node = graph->create_node("leia", "robot");
  auto bedroom =  graph->create_node("bedroom", "room");
  auto relation1 = node->add_relation("is", bedroom);
  auto relation2 = node->add_relation("isbis", bedroom);

  geometry_msgs::TransformStamped tf;
  tf.header.seq = 1;
  tf.header.frame_id = "/world";
  tf.child_frame_id = "/robot";
  tf.transform.translation.x = 1;
  tf.transform.translation.y = 1;
  tf.transform.translation.z = 1;
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;

  auto relation_tf = node->add_tf_relation(tf, bedroom);

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
  relation_tf.transform.header.seq = 1;
  relation_tf.transform.header.frame_id = "/world";
  relation_tf.transform.child_frame_id = "/robot";
  relation_tf.transform.transform.translation.x = 1;
  relation_tf.transform.transform.translation.y = 1;
  relation_tf.transform.transform.translation.z = 1;
  relation_tf.transform.transform.rotation.x = 0;
  relation_tf.transform.transform.rotation.y = 0;
  relation_tf.transform.transform.rotation.z = 0;
  relation_tf.transform.transform.rotation.w = 1;

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

  geometry_msgs::TransformStamped tf;
  tf.header.seq = 1;
  tf.header.frame_id = "/world";
  tf.child_frame_id = "/robot";
  tf.transform.translation.x = 1;
  tf.transform.translation.y = 1;
  tf.transform.translation.z = 1;
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;

  auto g_relation_tf = g_node->add_tf_relation(tf, g_bedroom);

  ASSERT_EQ(*graph_msg, *graph);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
