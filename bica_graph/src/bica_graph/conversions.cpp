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

#include <bica_graph/conversions.h>
#include <bica_graph/graph.h>
#include <bica_graph/node.h>
#include <bica_graph/relation.h>
#include <bica_graph/tf_relation.h>

#include <bica_msgs/Node.h>

bica_msgs::GraphConstPtr
bica_graph::graph_to_msg(const bica_graph::BicaGraph& graph)
{
  bica_msgs::GraphPtr msg (new bica_msgs::Graph());

  for (auto node : graph.get_nodes())
  {
    bica_msgs::NodePtr node_msg (new bica_msgs::Node());
    for (auto relation : node->get_relations())
    {
      relation->add_to_msg(node_msg);
    }

    node_msg->id = node->get_id();
    node_msg->type = node->get_type();
    node_msg->stamp = node->get_time_stamp();

    msg->nodes.push_back(*node_msg);
  }

  return msg;
}

bica_graph::BicaGraph::SharedPtr
bica_graph::msg_to_graph(const bica_msgs::GraphConstPtr& msg)
{
  auto graph = std::make_shared<bica_graph::BicaGraph>();

  for (int i = 0; i < msg->nodes.size(); i++)
  {
    auto node = graph->create_node(msg->nodes[i].id, msg->nodes[i].type,
      msg->nodes[i].stamp);
  }

  for (int i = 0; i < msg->nodes.size(); i++)
  {
    auto node = graph->get_node(msg->nodes[i].id);
    node->add_relations_from_msg(msg->nodes[i], graph);
  }


  return graph;
}
