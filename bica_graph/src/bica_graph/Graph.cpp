// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <map>
#include <algorithm>
#include <sstream>
#include <regex>
#include <vector>

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

boost::optional<Node>
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

  if (edges) {
    return std::find(edges.value()->begin(), edges.value()->end(), edge) != edges.value()->end();
  } else {
    return false;
  }
}

boost::optional<std::vector<Edge> *>
Graph::get_edges(const std::string & source, const std::string & target)
{
  ConnectionT connection {source, target};
  if (exist_node(source) && exist_node(target) && edges_.find(connection) != edges_.end()) {
    return &edges_[connection];
  } else {
    return {};
  }
}

std::string
Graph::to_string() const
{
  std::ostringstream graphstr(std::ostringstream::ate);

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
Graph::get_node_names_by_id(const std::string & expr)
{
  std::vector<std::string> ret;

  for (auto node : nodes_) {
    if (std::regex_match(node.first, std::regex(expr))) {
      ret.push_back(node.first);
    }
  }

  return ret;
}

std::vector<std::string>
Graph::get_node_names_by_type(const std::string & type)
{
  std::vector<std::string> ret;

  for (auto node : nodes_) {
    if (node.second.type == type) {
      ret.push_back(node.first);
    }
  }

  return ret;
}

std::vector<Edge>
Graph::get_edges_from_node(const std::string & node_src_id, const std::string & type)
{
  std::vector<Edge> ret;

  for (auto pair_nodes : edges_) {
    if (pair_nodes.first.first == node_src_id) {
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
Graph::get_edges_from_node_by_data(
  const std::string & node_src_id,
  const std::string & expr,
  const std::string & type)
{
  std::vector<Edge> ret;

  for (auto pair_nodes : edges_) {
    if (pair_nodes.first.first == node_src_id) {
      for (auto edge : pair_nodes.second) {
        if (((edge.type == type) || (type == "")) &&
          std::regex_match(edge.content, std::regex(expr)))
        {
          ret.push_back(edge);
        }
      }
    }
  }

  return ret;
}

std::vector<Edge>
Graph::get_edges_by_data(const std::string & expr, const std::string & type)
{
  std::vector<Edge> ret;

  for (auto pair_nodes : edges_) {
    for (auto edge : pair_nodes.second) {
      if (((edge.type == type) || (type == "")) &&
        std::regex_match(edge.content, std::regex(expr)))
      {
        ret.push_back(edge);
      }
    }
  }

  return ret;
}

}  // namespace bica_graph
