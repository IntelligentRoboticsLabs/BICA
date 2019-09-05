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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <map>

#include "bica_graph/graph.h"

namespace bica_graph
{

Graph::Graph(const ros::Time& ts)
: ts_(ts)
{
  ts_ = ts;
}

bool
Graph::exist_node(const std::string& id) const
{
  return std::find(nodes_.begin(), nodes_.end(), id) != nodes_.end();
}

void
Graph::add_node(const Node& other)
{
  add_node(other.get_id(), other.get_type());
}

void
Graph::add_node(const std::string& id, const std::string& type)
{
  if (!exist_node(id))
  {
    nodes_.push_back(Node(id, type));
    ts_ = ros::Time::now();
  }
  else
  {
    const Node& node = get_node(id);
    if (node.get_type() != type)
    {
      throw bica_graph::exceptions::NodeTypeMismatch(
        "Trying to update a node with same id ["+id+"] and different type ["+
        type +" != "+ node.get_type()+"]");
    }
    ts_ = ros::Time::now();
  }
}

const Node&
Graph::get_node(const std::string& id)
{
  for (const Node& node : nodes_)
  {
    if (node.get_id() == id) return node;
  }

  throw bica_graph::exceptions::NodeNotFound("Node not found [" + id + "]");
}

void
Graph::remove_node(const std::string& node_id)
{
  if (!exist_node(node_id))
  {
    throw bica_graph::exceptions::NodeNotFound("Node not found [" + node_id + "] removing node");
  }

  auto it = std::find(nodes_.begin(), nodes_.end(), node_id);
  nodes_.erase(it);

  // Check String edges
  auto str_it = string_edges_.begin();
  while (str_it != string_edges_.end())
  {
    if ((str_it->get_source() == node_id) || (str_it->get_target() == node_id))
    {
      str_it = string_edges_.erase(str_it);
      ts_ = ros::Time::now();
    }
    else
    {
      str_it++;
    }
  }
  // Check Double edges
  auto bdl_it = double_edges_.begin();
  while (bdl_it != double_edges_.end())
  {
    if ((bdl_it->get_source() == node_id) || (bdl_it->get_target() == node_id))
    {
      bdl_it = double_edges_.erase(bdl_it);
      ts_ = ros::Time::now();
    }
    else
    {
      bdl_it++;
    }
  }
  // Check TF edges
  auto tf_it = tf_edges_.begin();
  while (tf_it != tf_edges_.end())
  {
    if ((tf_it->get_source() == node_id) || (tf_it->get_target() == node_id))
    {
      tf_it = tf_edges_.erase(tf_it);
      ts_ = ros::Time::now();
    }
    else
    {
      tf_it++;
    }
  }
}

size_t
Graph::count_nodes() const
{
  return nodes_.size();
}

void
Graph::add_edge(const std::string& source, const std::string& data, const std::string& target)
{
  add_edge(StringEdge(source, data, target));
}

void
Graph::add_edge(const std::string& source, const double data, const std::string& target)
{
  add_edge(DoubleEdge(source, data, target));
}

void
Graph::add_edge(const std::string& source, const tf2::Transform& data, const std::string& target, bool static_tf)
{
  add_edge(TFEdge(source, data, target, static_tf));
}

void
Graph::add_edge(const StringEdge& other)
{
  if (!exist_node(other.get_source()))
    throw bica_graph::exceptions::NodeNotFound(
      "Adding edge not possible: source [" + other.get_source() + "] not found");

  if (!exist_node(other.get_target()))
    throw bica_graph::exceptions::NodeNotFound(
      "Adding edge not possible: target [" + other.get_target() + "] not found");

  if (!exist_edge(other))
    string_edges_.push_back(other);
}

void
Graph::add_edge(const DoubleEdge& other)
{
  if (!exist_node(other.get_source()))
    throw bica_graph::exceptions::NodeNotFound(
      "Adding edge not possible: source [" + other.get_source() + "] not found");

  if (!exist_node(other.get_target()))
    throw bica_graph::exceptions::NodeNotFound(
      "Adding edge not possible: target [" + other.get_target() + "] not found");

  if (!exist_edge(other))
    double_edges_.push_back(other);
}

void
Graph::add_edge(const TFEdge& other)
{
  if (!exist_node(other.get_source()))
    throw bica_graph::exceptions::NodeNotFound(
      "Adding edge not possible: source [" + other.get_source() + "] not found");

  if (!exist_node(other.get_target()))
    throw bica_graph::exceptions::NodeNotFound(
      "Adding edge not possible: target [" + other.get_target() + "] not found");

  if (!exist_edge(other))
    tf_edges_.push_back(other);
}

void
Graph::add_tf_edge(const std::string& source, const std::string& target, bool static_tf)
{
  add_edge(TFEdge(source, target, static_tf));
}

bool
Graph::exist_edge(const std::string& source, const std::string& data, const std::string& target)
{
  return exist_edge(StringEdge(source, data, target));
}

bool
Graph::exist_edge(const StringEdge& other)
{
  return std::find(string_edges_.begin(), string_edges_.end(), other) != string_edges_.end();
}

bool
Graph::exist_edge(const DoubleEdge& other)
{
  return std::find(double_edges_.begin(), double_edges_.end(), other) != double_edges_.end();
}

bool
Graph::exist_edge(const TFEdge& other)
{
  return std::find(tf_edges_.begin(), tf_edges_.end(), other) != tf_edges_.end();
}

bool
Graph::exist_tf_edge(const std::string& source, const std::string& target)
{
  return exist_edge(TFEdge(source, target));
}

bool
Graph::exist_double_edge(const std::string& source, const std::string& target)
{
  return exist_edge(DoubleEdge(source, target));
}

void
Graph::remove_edge(const std::string& source, const std::string& data, const std::string& target)
{
  remove_edge(StringEdge(source, data, target));
}

void
Graph::remove_tf_edge(const std::string& source, const std::string& target)
{
  remove_edge(TFEdge(source, target));
}

void
Graph::remove_double_edge(const std::string& source, const std::string& target)
{
  remove_edge(DoubleEdge(source, target));
}

void
Graph::remove_edge(const StringEdge& other)
{
  if (exist_edge(other))
  {
    auto it = std::find(string_edges_.begin(), string_edges_.end(), other);
    string_edges_.erase(it);
  }
}

void
Graph::remove_edge(const DoubleEdge& other)
{
  if (exist_edge(other))
  {
    auto it = std::find(double_edges_.begin(), double_edges_.end(), other);
    double_edges_.erase(it);
  }
}

void
Graph::remove_edge(const TFEdge& other)
{
  if (exist_edge(other))
  {
    auto it = std::find(tf_edges_.begin(), tf_edges_.end(), other);
    tf_edges_.erase(it);
  }
}


DoubleEdge&
Graph::get_double_edge(const std::string& source, const std::string& target)
{
  DoubleEdge edge_to_search(source, target);
  if (exist_edge(edge_to_search))
  {
    auto it = std::find(double_edges_.begin(), double_edges_.end(), edge_to_search);
    return *it;
  }
  else
    throw bica_graph::exceptions::EdgeNotFound(
      "Double Edge not found:  [" + source + "] ->  [" + target + "]");
}

TFEdge&
Graph::get_tf_edge(const std::string& source, const std::string& target)
{
  TFEdge edge_to_search(source, target);
  if (exist_edge(edge_to_search))
  {
    auto it = std::find(tf_edges_.begin(), tf_edges_.end(), edge_to_search);
    return *it;
  }
  else
    throw bica_graph::exceptions::EdgeNotFound(
      "TF Edge not found:  [" + source + "] ->  [" + target + "]");
}

void
Graph::print()
{
  std::cout << "=============================================" << std::endl;
  for (auto node : nodes_)
  {
    std::cout << "Node [" << node.get_id() << "] (" << node.get_type() << ")" << std::endl;
  }

  for (auto edge : string_edges_)
  {
    std::cout << "Edge (" << edge.get_source() << ")---[" << edge.get()
      << "]--->(" << edge.get_target() << ")" << std::endl;
  }

  for (auto edge : double_edges_)
  {
    std::cout << "Edge (" << edge.get_source() << ")---[" << edge.get()
      << "]--->(" << edge.get_target() << ")" << std::endl;
  }

  for (auto edge : tf_edges_)
  {
    std::cout << "Edge (" << edge.get_source() << ")---[(" <<
      edge.get().getOrigin().x() << ", " <<
      edge.get().getOrigin().y() << ", " <<
      edge.get().getOrigin().z() <<
      ")]--->(" << edge.get_target() << ")" << std::endl;
  }

  std::cout << "=============================================" << std::endl;
}

bool operator==(const Graph& lhs, const Graph& rhs)
{
  return lhs.nodes_ == rhs.nodes_ && lhs.string_edges_ == rhs.string_edges_ &&
    lhs.double_edges_ == rhs.double_edges_ && lhs.tf_edges_ == rhs.tf_edges_;
}


}  // namespace bica_graph
