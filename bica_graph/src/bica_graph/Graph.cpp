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
#include <algorithm>
#include <sstream>
#include <regex>

#include "bica_graph/Graph.hpp"

namespace bica_graph
{

Graph::Graph()
{
}

bool
Graph::add_node(const Node & node)
{
  if (nodes_.find(node.name) != nodes_.end()) {
    return nodes_[node.name].type == node.type;
  } else {
    nodes_[node.name] = node;
    return true;
  }
}

bool
Graph::remove_node(const std::string node)
{
  if (nodes_.find(node) != nodes_.end()) {
    nodes_.erase(node);

    auto it = edges_.begin();
    while (it != edges_.end()) {
      if (it->first.first == node || it->first.second == node) {
        it = edges_.erase(it);
      } else {
        ++it;
      }
    }
    return true;
  } else {
    return false;
  }
}

bool
Graph::exist_node(const std::string node)
{
  return nodes_.find(node) != nodes_.end();
}

std::optional<Node>
Graph::get_node(const std::string node)
{
  if (exist_node(node)) {
    return nodes_[node];
  } else {
    return {};
  }
}

bool
Graph::add_edge(const Edge & edge)
{
  if (exist_node(edge.source) && exist_node(edge.target)) {
    if (!exist_edge(edge)) {
      ConnectionT connection {edge.source, edge.target};
      edges_[connection].push_back(edge);
    }
    return true;
  } else {
    return false;
  }
}

bool
Graph::remove_edge(const Edge & edge)
{
  if (exist_node(edge.source) && exist_node(edge.target) && exist_edge(edge)) {
    auto edges = get_edges(edge.source, edge.target);

    bool found = false;
    int i = 0;
    while (!found && i < edges.value()->size()) {
      if (edge == edges.value()->at(i)) {
        found = true;
        edges.value()->erase(edges.value()->begin() + i);
      }
      i++;
    }
    return found;
  } else {
    return false;
  }
}

bool
Graph::exist_edge(const Edge & edge)
{
  if (!exist_node(edge.source) || !exist_node(edge.target)) {
    return false;
  }

  auto edges = get_edges(edge.source, edge.target);

  if (edges.has_value()) {
    return std::find(edges.value()->begin(), edges.value()->end(), edge) != edges.value()->end();
  } else {
    return false;
  }
}

std::optional<std::vector<Edge>*>
Graph::get_edges(const std::string & source, const std::string & target)
{
  ConnectionT connection {source, target};
  if (exist_node(source) && exist_node(target) &&  edges_.find(connection) != edges_.end()) {
    return &edges_[connection];
  } else {
    return {};
  }
}

std::string
Graph::to_string() const
{
  std::ostringstream graphstr (std::ostringstream::ate);
  
  graphstr << "Nodes: " << nodes_.size() << std::endl;
  for (const auto & node : nodes_) {
    graphstr << node.second.to_string() << std::endl;
  }

  graphstr << "Edges: " << get_num_edges() << std::endl;
  for (const auto & nodes_pair : edges_) {
    for (const auto & edge : nodes_pair.second) {
      graphstr << edge.to_string() << std::endl; 
    }
  }

  return graphstr.str();
}

void
Graph::from_string(const std::string & graph_str)
{
  nodes_.clear();
  edges_.clear();

  std::istringstream graph_istr(graph_str);
  std::string line;
  while (std::getline(graph_istr, line)) {
    if (line.substr(0, 4) == "node") {
      Node node;
      node.from_string(line);
      add_node(node);
    } else if (line.substr(0, 4) == "edge") {
      Edge edge;
      edge.from_string(line);
      add_edge(edge);
    }
  }
}

size_t
Graph::get_num_edges() const
{
  size_t counter = 0;
  for (const auto & nodes_pair : edges_) {
    counter += nodes_pair.second.size();
  }
  return counter;
}

size_t
Graph::get_num_nodes() const
{
  return nodes_.size();
}

std::vector<std::string>
Graph::get_node_names_by_id(const std::string& expr)
{
  std::vector<std::string> ret;

  for (auto node : nodes_)
  {
    if (std::regex_match (node.first, std::regex(expr)))
    {
      ret.push_back(node.first);
    }
  }

  return ret;
}

std::vector<std::string>
Graph::get_node_names_by_type(const std::string& type)
{
  std::vector<std::string> ret;

  for (auto node : nodes_)
  {
    if (node.second.type == type) {
      ret.push_back(node.first);
    }
  }

  return ret;
}

std::vector<Edge>
Graph::get_edges_from_node(const std::string& node_src_id, const std::string& type)
{
  std::vector<Edge> ret;

  for (auto pair_nodes : edges_)
  {
    if (pair_nodes.first.first == node_src_id)
    {
      for (auto edge : pair_nodes.second) {
        if ((edge.type == type) || (type == "")) {
          ret.push_back(edge);
        }
      }
    }
  }

  return ret;
}

std::vector<Edge>
Graph::get_edges_from_node_by_data(const std::string& node_src_id, const std::string& expr, const std::string& type)
{
  std::vector<Edge> ret;

  for (auto pair_nodes : edges_)
  {
    if (pair_nodes.first.first == node_src_id)
    {
      for (auto edge : pair_nodes.second) {
        if (((edge.type == type) || (type == "")) && std::regex_match(edge.content, std::regex(expr))) {
          ret.push_back(edge);
        }
      }
    }
  }

  return ret;
}

std::vector<Edge>
Graph::get_edges_by_data(const std::string& expr, const std::string& type)
{
  std::vector<Edge> ret;

  for (auto pair_nodes : edges_)
  {
    for (auto edge : pair_nodes.second) {
      if (((edge.type == type) || (type == "")) && std::regex_match(edge.content, std::regex(expr))) {
        ret.push_back(edge);
      }
    }
  }

  return ret;
}

}  // namespace bica_graph
