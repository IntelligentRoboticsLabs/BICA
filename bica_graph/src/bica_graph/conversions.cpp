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
void node_to_msg(const Node::SharedPtr& node, bica_msgs::Node* msg)
{
  msg->id = node->get_id();
  msg->type = node->get_type();
}

void msg_to_node(const bica_msgs::Node& msg, Node::SharedPtr& node)
{
  node = std::make_shared<Node>(msg.id, msg.type);
}


const char* get_msg_type_string(uint type)
{
  switch (type)
  {
    case bica_msgs::Edge::EDGE_TYPE_STRING:
      return "EDGE_TYPE_STRING";
    case bica_msgs::Edge::EDGE_TYPE_DOUBLE:
      return "EDGE_TYPE_DOUBLE";
    case bica_msgs::Edge::EDGE_TYPE_TF:
      return "EDGE_TYPE_TF";
    case bica_msgs::Edge::EDGE_TYPE_UNKNOWN:
      return "EDGE_TYPE_UNKNOWN";
  };

  return "unknown";
}

void graph_to_msg(const bica_graph::Graph& graph, bica_msgs::Graph* msg)
{
  msg->stamp = graph.get_time_stamp();

  auto nodes = graph.get_nodes();
  for (auto it_nodes = nodes.begin(); it_nodes != nodes.end(); ++it_nodes)
  {
    bica_msgs::Node node_msg;
    node_msg.id = it_nodes->first;
    node_to_msg(it_nodes->second, &node_msg);

    msg->nodes.push_back(node_msg);
  }

  auto edges = graph.get_edges();
  for (auto it_edges = edges.begin(); it_edges != edges.end(); ++it_edges)
  {
    std::string source = it_edges->first.first;
    std::string target = it_edges->first.second;

    for (auto it_edge_type = it_edges->second.begin(); it_edge_type != it_edges->second.end(); ++it_edge_type)
    {
      bica_msgs::Edge edge_msg;
      edge_msg.source = source;
      edge_msg.target = target;

      try
      {
        switch ((*it_edge_type)->get_type())
        {
          case bica_graph::STRING:
            edge_to_msg<std::string>(*it_edge_type, &edge_msg);
            break;
          case bica_graph::DOUBLE:
            edge_to_msg<double>(*it_edge_type, &edge_msg);
            break;
          case bica_graph::TF:
            edge_to_msg<tf::Transform>(*it_edge_type, &edge_msg);
            break;
          default:
            throw bica_graph::exceptions::OperationNotValid("Unable to transform edge to msg");
        }
      }
      catch (bica_graph::exceptions::OperationNotValid& e)
      {
        ROS_ERROR("graph_to_msg:: %s --> %s : %s", source.c_str(), target.c_str(), e.what());
      }

      msg->edges.push_back(edge_msg);
    }
  }
}

void msg_to_graph(const bica_msgs::Graph& msg, bica_graph::Graph::SharedPtr& graph)
{
  graph = std::make_shared<bica_graph::Graph>(msg.stamp);

  for (auto node_it : msg.nodes)
  {
    bica_graph::Node::SharedPtr new_node;
    bica_graph::msg_to_node(node_it, new_node);
    graph->add_node(*new_node);
  }

  for (auto edges_it : msg.edges)
  {
    switch (edges_it.type)
    {
      case bica_msgs::Edge::EDGE_TYPE_STRING:
        {
          std::shared_ptr<bica_graph::Edge<std::string>> new_edge;
          bica_graph::msg_to_edge(edges_it, new_edge);
          graph->add_edge<std::string>(*new_edge);
        }
        break;
      case bica_msgs::Edge::EDGE_TYPE_DOUBLE:
        {
          std::shared_ptr<bica_graph::Edge<double>> new_edge;
          bica_graph::msg_to_edge(edges_it, new_edge);
          graph->add_edge<double>(*new_edge);
        }
        break;
      case bica_msgs::Edge::EDGE_TYPE_TF:
        {
          graph->add_tf_edge(edges_it.source, edges_it.target);
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

template<class T>
void edge_to_msg(const typename Edge<T>::SharedPtr& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_UNKNOWN;

  ROS_WARN("Using default edge_to_msg");
}

template<class T>
void msg_to_edge(const bica_msgs::Edge& msg, std::shared_ptr<bica_graph::Edge<T>>& edge)
{
  switch (msg.type)
  {
    case bica_msgs::Edge::EDGE_TYPE_UNKNOWN:
      throw bica_graph::exceptions::OperationNotValid(
        "Unable to transform msg of type EDGE_TYPE_UNKNOWN to egde");
      break;
    default:
      throw bica_graph::exceptions::OperationNotValid(
        "Unable to transform msg of type not yet added con conversions.h");
  };
}

template<>
void msg_to_edge<std::string>(const bica_msgs::Edge& msg, std::shared_ptr<bica_graph::Edge<std::string>>& edge)
{
  if (msg.type != bica_msgs::Edge::EDGE_TYPE_STRING)
  {
    char message[255];
    snprintf(message,  sizeof(message),
      "Unable to transform msg of type %s to egde std::string", get_msg_type_string(msg.type));
    throw bica_graph::exceptions::OperationNotValid(message);
  }

  edge = std::make_shared<bica_graph::Edge<std::string>>(
    msg.source, msg.target, msg.string_data);
}

template<>
void msg_to_edge<double>(const bica_msgs::Edge& msg, std::shared_ptr<bica_graph::Edge<double>>& edge)
{
  if (msg.type != bica_msgs::Edge::EDGE_TYPE_DOUBLE)
  {
    char message[255];
    snprintf(message,  sizeof(message),
      "Unable to transform msg of type %s to egde std::string", get_msg_type_string(msg.type));
    throw bica_graph::exceptions::OperationNotValid(message);
  }

  edge = std::make_shared<bica_graph::Edge<double>>(
    msg.source, msg.target, msg.double_data);
}

template<>
void msg_to_edge<tf::Transform>(const bica_msgs::Edge& msg, std::shared_ptr<bica_graph::Edge<tf::Transform>>& edge)
{
  if (msg.type != bica_msgs::Edge::EDGE_TYPE_TF)
  {
    char message[255];
    snprintf(message,  sizeof(message),
      "Unable to transform msg of type %s to egde std::string", get_msg_type_string(msg.type));
    throw bica_graph::exceptions::OperationNotValid(message);
  }

  edge = std::make_shared<bica_graph::Edge<tf::Transform>>(msg.source, msg.target);
}

template<>
void edge_to_msg<std::string>(const Edge<std::string>::SharedPtr& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_STRING;
  msg->source = edge->get_source();
  msg->target = edge->get_target();

  msg->string_data = std::dynamic_pointer_cast<bica_graph::Edge<std::string>>(edge)->get();
}

template<>
void edge_to_msg<double>(const Edge<double>::SharedPtr& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_DOUBLE;
  msg->source = edge->get_source();
  msg->target = edge->get_target();

  msg->double_data = std::dynamic_pointer_cast<bica_graph::Edge<double>>(edge)->get();
}

template<>
void edge_to_msg<tf::Transform>(const Edge<tf::Transform>::SharedPtr& edge, bica_msgs::Edge* msg)
{
  msg->type = bica_msgs::Edge::EDGE_TYPE_TF;
  msg->source = edge->get_source();
  msg->target = edge->get_target();
}


}  // namespace bica_graph
