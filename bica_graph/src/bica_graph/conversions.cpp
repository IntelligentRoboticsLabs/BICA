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

#include "bica_graph/conversions.h"

namespace bica_graph
{
void node_to_msg(const Node& node, bica_msgs::Node* msg)
{
  msg->id = node.get_id();
  msg->type = node.get_type();
}

void msg_to_node(const bica_msgs::Node& msg, Node* node)
{
  node->set_id(msg.id);
  node->set_type(msg.type);
}


void graph_to_msg(const bica_graph::Graph& graph, bica_msgs::Graph* msg)
{
  msg->stamp = graph.get_time_stamp();
  msg->responsable_id = graph.get_responsable_id();

  for (auto node : graph.get_nodes())
  {
    bica_msgs::Node node_msg;
    node_msg.id = node.get_id();
    node_msg.type = node.get_type();

    msg->nodes.push_back(node_msg);
  }

  for (auto edge : graph.get_string_edges())
  {
    bica_msgs::Edge edge_msg;
    edge_to_msg(edge, &edge_msg);

    msg->edges.push_back(edge_msg);
  }

  for (auto edge : graph.get_double_edges())
  {
    bica_msgs::Edge edge_msg;
    edge_to_msg(edge, &edge_msg);

    msg->edges.push_back(edge_msg);
  }

  for (auto edge : graph.get_tf_edges())
  {
    bica_msgs::Edge edge_msg;
    edge_to_msg(edge, &edge_msg);

    msg->edges.push_back(edge_msg);
  }
}

void msg_to_graph(const bica_msgs::Graph& msg, bica_graph::Graph::SharedPtr& graph)
{
  graph = std::make_shared<bica_graph::Graph>(msg.stamp);
  graph->set_responsable_id(msg.responsable_id);

  for (auto node_it : msg.nodes)
  {
    graph->add_node(bica_graph::Node(node_it.id, node_it.type));
  }

  for (auto edges_it : msg.edges)
  {
    switch (edges_it.type)
    {
      case bica_msgs::Edge::EDGE_TYPE_STRING:
        {
          graph->add_edge(bica_graph::StringEdge(edges_it.source, edges_it.string_data, edges_it.target));
        }
        break;
      case bica_msgs::Edge::EDGE_TYPE_DOUBLE:
        {
          graph->add_edge(bica_graph::DoubleEdge(edges_it.source, edges_it.double_data, edges_it.target));
        }
        break;
      case bica_msgs::Edge::EDGE_TYPE_TF:
        {
          graph->add_edge(bica_graph::TFEdge(edges_it.source, edges_it.target, edges_it.static_tf));
        }
        break;
      case bica_msgs::Edge::EDGE_TYPE_UNKNOWN:
        throw bica_graph::exceptions::OperationNotValid(
          "Unable to transform msg of type EDGE_TYPE_UNKNOWN to egde reading a mgs");
        break;
      default:
        throw bica_graph::exceptions::OperationNotValid(
          "Unable to transform msg of type not yet added con conversions.h reading a mgs");
      };
  }
}


void edge_to_msg(const bica_graph::StringEdge& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_STRING;
  msg->source = edge.get_source();
  msg->target = edge.get_target();
  msg->string_data = edge.get();
}

void edge_to_msg(const bica_graph::DoubleEdge& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_DOUBLE;
  msg->source = edge.get_source();
  msg->target = edge.get_target();
  msg->double_data = edge.get();
}

void edge_to_msg(const bica_graph::TFEdge& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_TF;
  msg->source = edge.get_source();
  msg->target = edge.get_target();
  msg->static_tf = edge.is_static();
}

}  // namespace bica_graph
