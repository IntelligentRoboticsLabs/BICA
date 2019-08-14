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

  bool exist_node(const std::string& id) const;
  void add_node(const Node& other);
  void add_node(const std::string& id, const std::string& type);
  const Node& get_node(const std::string& id);
  void remove_node(const std::string& node_id);

  size_t count_nodes() const;

  void add_edge(const std::string& source, const std::string& data, const std::string& target);
  void add_edge(const std::string& source, const double data, const std::string& target);
  void add_edge(const std::string& source, const tf2::Transform& data,
    const std::string& target, bool static_tf = false);
  void add_edge(const StringEdge& other);
  void add_edge(const DoubleEdge& other);
  void add_edge(const TFEdge& other);
  void add_tf_edge(const std::string& source, const std::string& target, bool static_tf = false);

  bool exist_edge(const std::string& source, const std::string& data, const std::string& target);
  bool exist_edge(const StringEdge& other);
  bool exist_edge(const DoubleEdge& other);
  bool exist_edge(const TFEdge& other);
  bool exist_tf_edge(const std::string& source, const std::string& target);
  bool exist_double_edge(const std::string& source, const std::string& target);

  void remove_edge(const std::string& source, const std::string& data, const std::string& target);
  void remove_edge(const StringEdge& other);
  void remove_edge(const DoubleEdge& other);
  void remove_edge(const TFEdge& other);
  void remove_tf_edge(const std::string& source, const std::string& target);
  void remove_double_edge(const std::string& source, const std::string& target);

  DoubleEdge& get_double_edge(const std::string& source, const std::string& target);
  TFEdge& get_tf_edge(const std::string& source, const std::string& target);

  const std::list<Node>& get_nodes() const {return nodes_;}
  const std::list<StringEdge>& get_string_edges() const {return string_edges_;}
  const std::list<DoubleEdge>& get_double_edges() const {return double_edges_;}
  const std::list<TFEdge>& get_tf_edges() const {return tf_edges_;}

  ros::Time get_time_stamp() const {return ts_;}
  void update_time_stamp() {ts_ = ros::Time::now();}
  void set_responsable_id(const std::string& responsable_id) {responsable_id_ = responsable_id;}
  std::string get_responsable_id() const {return responsable_id_;}
  void print();

  friend bool operator==(const Graph& lhs, const Graph& rhs);
private:
  bool check_source_target(const std::string& source, const std::string& target) const;

  std::list<Node> nodes_;

  std::list<StringEdge> string_edges_;
  std::list<DoubleEdge> double_edges_;
  std::list<TFEdge> tf_edges_;

  ros::Time ts_;
  std::string responsable_id_;
};


}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_H
