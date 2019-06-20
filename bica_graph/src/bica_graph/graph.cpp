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

#include "bica_graph/graph.h"

namespace bica_graph
{

Graph::Graph(const ros::Time& ts)
: ts_(ts)
{
  ts_ = ts;
}

void
Graph::add_node(const std::string& id, const std::string& type)
{
  auto node = nodes_.find(id);

  if (node == nodes_.end())
  {
    nodes_[id] = std::make_shared<Node>(id, type);
    ts_ = ros::Time::now();
  }
  else
  {
    if ((node->second)->get_type() != type)
    {
      throw bica_graph::exceptions::NodeTypeMismatch(
        "Trying to update a node with same id ["+id+"] and different type ["+
        type +" != "+ (node->second)->get_type()+"]");
    }

    ts_ = ros::Time::now();
  }
}

void
Graph::add_node(const bica_graph::Node& node)
{
  add_node(node.get_id(), node.get_type());
}

Node::SharedPtr
Graph::get_node(const std::string& id)
{
  auto node = nodes_.find(id);

  if (node == nodes_.end())
  {
    throw bica_graph::exceptions::NodeNotFound("Node not found [" + id + "]");
  }

  return node->second;
}

Node::ConstSharedPtr
Graph::get_const_node(const std::string& id) const
{
  auto node = nodes_.find(id);

  if (node == nodes_.end())
  {
    throw bica_graph::exceptions::NodeNotFound("Node not found [" + id + "]");
  }

  return node->second;
}

size_t
Graph::count_nodes() const
{
  return nodes_.size();
}

bool
Graph::exist_node(const std::string& id) const
{
  return (nodes_.find(id) != nodes_.end());
}

void
Graph::add_tf_edge(const std::string& source, const std::string& target)
{
  std::pair<std::string, std::string> idx(source, target);

  auto edge = edges_.find(idx);
  if (edge == edges_.end())
  {
    edges_[idx] = std::list<EdgeBase::SharedPtr>();
    edges_[idx].push_back(std::make_shared<Edge<tf::Transform>>(source, target));

    ts_ = ros::Time::now();
  }
  else
  {
    auto edge_typed = std::dynamic_pointer_cast<bica_graph::Edge<tf::Transform>>(
        get_edge<tf::Transform>(source, target));

    if (edge_typed == nullptr)
    {
      edges_[idx].push_back(std::make_shared<Edge<tf::Transform>>(source, target));
      ts_ = ros::Time::now();
    }
  }
}

size_t
Graph::count_edges(const std::string& source, const std::string& target) const
{
  try
  {
    check_source_target(source, target);
  } catch(bica_graph::exceptions::NodeNotFound& e)
  {
    return 0;
  }

  std::pair<std::string, std::string> idx(source, target);

  auto edge = edges_.find(idx);

  if (edge == edges_.end())
  {
    return 0;
  }
  else
  {
    return edge->second.size();
  }

}

void
Graph::remove_node(const std::string& node_id)
{
  if (!exist_node(node_id))
  {
    throw bica_graph::exceptions::NodeNotFound("Node not found [" + node_id + "] removing node");
  }

  nodes_.erase(nodes_.find(node_id));

  auto it = edges_.begin();

  while (it != edges_.end())
  {
    if (it->first.first == node_id || it->first.second == node_id)
    {
      it = edges_.erase(it);
      ts_ = ros::Time::now();
    }
    else
    {
      it++;
    }
  }
}

const std::map<std::string, Node::SharedPtr>&
Graph::get_nodes() const
{
  return nodes_;
}

const std::map<std::pair<std::string, std::string>, std::list<EdgeBase::SharedPtr>>&
Graph::get_edges() const
{
  return edges_;
}

ros::Time
Graph::get_time_stamp() const
{
  return ts_;
}

void
Graph::print()
{
  std::cout<<"============================================="<<std::endl;
  for (auto node : nodes_)
  {
    std::cout << "Node ["<<node.first<<"] ("<<node.second->get_type()<<")"<<std::endl;
  }
  for (auto edges : edges_)
  {
    for (auto edge : edges.second)
    {
      switch (edge->get_type())
      {
        case STRING:
            std::cout << "Edge ("<<edge->get_source()<<")---["<<edge->get<std::string>()<<"]--->("<<edge->get_target()<<")"<<std::endl;
            break;
        case DOUBLE:
            std::cout << "Edge ("<<edge->get_source()<<")---["<<edge->get<double>()<<"]--->("<<edge->get_target()<<")"<<std::endl;
            break;
        case TF:
            {
              tf::Transform tf_aux = edge->get<tf::Transform>();
              std::cout << "Edge ("<<edge->get_source()<<")---[("<<
                tf_aux.getOrigin().x()<<", "<<
                tf_aux.getOrigin().y()<<", "<<
                tf_aux.getOrigin().z()<<
                ")]--->("<<edge->get_target()<<")"<<std::endl;
            }
            break;
        default:
          std::cout << "Edge ("<<edge->get_source()<<")---[UNKNOWN]--->("<<edge->get_target()<<")"<<std::endl;
      }
    }

  }
  std::cout<<"============================================="<<std::endl;
}

bool
Graph::check_source_target(const std::string& source, const std::string& target) const
{
  if (!exist_node(source))
  {
    throw bica_graph::exceptions::NodeNotFound("Node not found [" + source + "] getting relation ["
      + source + " --> " + target + "]");
  }
  if (!exist_node(target))
  {
    throw bica_graph::exceptions::NodeNotFound("Node not found [" + target + "] getting relation ["
      + source + " --> " + target + "]");
  }
}

bool operator==(const Graph& lhs, const Graph& rhs)
{
  size_t lhs_count_nodes = lhs.count_nodes();
  size_t rhs_count_nodes = rhs.count_nodes();

  if (lhs_count_nodes != rhs_count_nodes)
    return false;

  for (auto node_lhs : lhs.nodes_)
  {
    try
    {
      if (!(*node_lhs.second == *rhs.get_const_node(node_lhs.second->get_id())))
        return false;
    }
    catch (bica_graph::exceptions::NodeNotFound& e)
    {
      return false;
    }
  }


  for (auto edge_lhs : lhs.edges_)
  {
    size_t lhs_count_edges = lhs.count_edges(edge_lhs.first.first, edge_lhs.first.second);
    size_t rhs_count_edges = rhs.count_edges(edge_lhs.first.first, edge_lhs.first.second);

    if (lhs_count_edges != rhs_count_edges)
      return false;

    // ToDo (fmrico): check every edge
  }

  return true;
}

}  // namespace bica_graph
