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

#ifndef BICA_GRAPH__GRAPH_HPP_
#define BICA_GRAPH__GRAPH_HPP_

#include <vector>
#include <map>
#include <string>
#include <utility>

#include "bica_graph/Types.hpp"
#include "bica_graph/GraphInterface.hpp"

namespace bica_graph
{

using ConnectionT = std::pair<std::string, std::string>;

class Graph : public GraphInterface
{
public:
  Graph();

  bool add_node(const Node & node);
  bool remove_node(const std::string node);
  bool exist_node(const std::string node);
  std::optional<Node> get_node(const std::string node);

  bool add_edge(const Edge & edge);
  bool remove_edge(const Edge & edge);
  bool exist_edge(const Edge & edge);

  std::optional<std::vector<Edge> *> get_edges(
    const std::string & source,
    const std::string & target);

  std::string to_string() const;
  void from_string(const std::string & graph_str);

  const std::map<std::string, Node> & get_nodes() {return nodes_;}
  const std::map<ConnectionT, std::vector<Edge>> & get_edges() {return edges_;}

  size_t get_num_edges() const;
  size_t get_num_nodes() const;

  std::vector<std::string> get_node_names_by_id(const std::string & expr);
  std::vector<std::string> get_node_names_by_type(const std::string & type);
  std::vector<Edge> get_edges_from_node(
    const std::string & node_src_id,
    const std::string & type = "");
  std::vector<Edge> get_edges_from_node_by_data(
    const std::string & node_src_id, const std::string & expr,
    const std::string & type = "");
  std::vector<Edge> get_edges_by_data(const std::string & expr, const std::string & type = "");

private:
  std::map<std::string, Node> nodes_;
  std::map<ConnectionT, std::vector<Edge>> edges_;
};


}  // namespace bica_graph

#endif  // BICA_GRAPH__GRAPH_HPP_
