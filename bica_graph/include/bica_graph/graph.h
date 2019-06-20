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

#ifndef BICA_GRAPH_GRAPH_H
#define BICA_GRAPH_GRAPH_H

#include <list>
#include <map>
#include <memory>
#include <string>
#include <iostream>
#include <utility>

#include <bica_graph/macros.h>
#include <bica_graph/exceptions.h>

#include <bica_graph/node.h>
#include <bica_graph/edge.h>

namespace bica_graph
{

class Graph
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(Graph)

  explicit Graph(const ros::Time& ts = ros::Time::now());

  void add_node(const std::string& id, const std::string& type);
  void add_node(const bica_graph::Node& node);

  Node::SharedPtr get_node(const std::string& id);
  Node::ConstSharedPtr get_const_node(const std::string& id) const;

  size_t count_nodes() const;

  bool exist_node(const std::string& id) const;

  void add_tf_edge(const std::string& source, const std::string& target);
  size_t count_edges(const std::string& source, const std::string& target) const;
  void remove_node(const std::string& node_id);

  const std::map<std::string, Node::SharedPtr>& get_nodes() const;

  const std::map<std::pair<std::string, std::string>, std::list<EdgeBase::SharedPtr>>&
  get_edges() const;

  ros::Time get_time_stamp() const;

  void print();

  template<class T>
  void
  add_edge(const Edge<T>& edge)
  {
    add_edge<T>(edge.get_source(), edge.get(), edge.get_target());
  }

  template<class T>
  bool
  exist_edge(const std::string& source, const std::string& target, const T& data = T()) const
  {
    return get_const_edge<T>(source, target, data) != nullptr;
  }


  template<class T>
  std::shared_ptr<Edge<T>>
  get_edge(const std::string& source, const std::string& target, const T& data = T())
  {
    typename Edge<T>::SharedPtr ret = nullptr;

    try
    {
      check_source_target(source, target);
    }
    catch(bica_graph::exceptions::NodeNotFound& e)
    {
      return nullptr;
    }

    std::pair<std::string, std::string> idx(source, target);

    auto edge = edges_.find(idx);

    EdgeType type_search = to_type<T>();
    if (edge != edges_.end())
    {
      for (std::list<EdgeBase::SharedPtr>::iterator it = edge->second.begin(); it!= edge->second.end(); ++it)
      {
        if ((*it)->get_type() != type_search)
          continue;

        if ((*it)->get_type() != TF)
        {
          auto comp_edge = std::make_shared<Edge<T>>(source, target, data);
          if (**it == *comp_edge)
            ret = *it;
        }
        else
        {
          if ((*it)->get_source() == source && (*it)->get_target() == target)
            ret = *it;
        }
      }
    }

    return std::dynamic_pointer_cast<bica_graph::Edge<T>>(ret);
  }

  template<class T>
  const std::shared_ptr<Edge<T>>
  get_const_edge(const std::string& source, const std::string& target, const T& data = T()) const
  {
    typename Edge<T>::SharedPtr ret = nullptr;

    try
    {
      check_source_target(source, target);
    }
    catch(bica_graph::exceptions::NodeNotFound& e)
    {
      return nullptr;
    }

    std::pair<std::string, std::string> idx(source, target);

    auto edge = edges_.find(idx);

    EdgeType type_search = to_type<T>();
    if (edge != edges_.end())
    {
      for (std::list<EdgeBase::SharedPtr>::const_iterator it = edge->second.begin(); it!= edge->second.end(); ++it)
      {
        if ((*it)->get_type() != type_search)
          continue;

        if ((*it)->get_type() != TF)
        {
          auto comp_edge = std::make_shared<Edge<T>>(source, target, data);
          if (**it == *comp_edge)
            ret = *it;
        }
        else
        {
          if ((*it)->get_source() == source && (*it)->get_target() == target)
            ret = *it;
        }
      }
    }

    return std::dynamic_pointer_cast<bica_graph::Edge<T>>(ret);
  }

  template<class T>
  void
  remove_edge(const std::string& source, const std::string& target, const T& data = T())
  {
    check_source_target(source, target);

    std::pair<std::string, std::string> idx(source, target);

    auto edge = edges_.find(idx);

    if (edge != edges_.end())
    {
      auto it = edge->second.begin();
      while (it!= edge->second.end())
      {
        auto edge_typed = std::dynamic_pointer_cast<Edge<T>>(*it);
        if (edge_typed != nullptr)
        {
          if (std::is_same<T, std::string>::value)
          {
            if (edge_typed->get() == data)
            {
              it = edge->second.erase(it);
              ts_ = ros::Time::now();
            }
            else
              ++it;
          }
          else
          {
            it = edge->second.erase(it);
            ts_ = ros::Time::now();
          }
        }
        else
          ++it;
      }
    }
  }

  template<class T>
  void
  add_edge(const std::string& source, const T& data, const std::string& target)
  {
    std::pair<std::string, std::string> idx(source, target);

    auto edge = edges_.find(idx);
    if (edge == edges_.end())
    {
      edges_[idx] = std::list<EdgeBase::SharedPtr>();
      edges_[idx].push_back(std::make_shared<Edge<T>>(source, target, data));

      ts_ = ros::Time::now();
    }
    else
    {
      auto edge_typed = std::dynamic_pointer_cast<bica_graph::Edge<T>>(
          get_edge<T>(source, target, data));

      if (edge_typed == nullptr)
      {
        edges_[idx].push_back(std::make_shared<Edge<T>>(source, target, data));
        ts_ = ros::Time::now();
      }
      else
      {
        edge_typed->set(data);
        ts_ = ros::Time::now();
      }
    }
  }

  friend bool operator==(const Graph& lhs, const Graph& rhs);

private:
  bool check_source_target(const std::string& source, const std::string& target) const;

  std::map<std::string, Node::SharedPtr> nodes_;
  std::map<std::pair<std::string, std::string>, std::list<EdgeBase::SharedPtr>> edges_;

  ros::Time ts_;
};


}  // namespace bica_graph

#include <bica_graph/conversions.h>

#endif  // BICA_GRAPH_GRAPH_H
