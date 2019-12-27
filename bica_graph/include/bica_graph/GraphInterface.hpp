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

#ifndef BICA_GRAPH__GRAPHINTERFACE_HPP_
#define BICA_GRAPH__GRAPHINTERFACE_HPP_

#include <vector>
#include <map>
#include <string>
#include <utility>

#include "bica_graph/Types.hpp"

namespace bica_graph
{

using ConnectionT = std::pair<std::string, std::string>;

class GraphInterface
{
public:
  virtual bool add_node(const Node & node) = 0;
  virtual bool remove_node(const std::string node) = 0;
  virtual bool exist_node(const std::string node) = 0;
  virtual std::optional<Node> get_node(const std::string node) = 0;

  virtual bool add_edge(const Edge & edge) = 0;
  virtual bool remove_edge(const Edge & edge) = 0;
  virtual bool exist_edge(const Edge & edge) = 0;

  virtual const std::map<ConnectionT, std::vector<Edge>> & get_edges() = 0;
  virtual const std::map<std::string, Node> & get_nodes() = 0;

  virtual std::optional<std::vector<Edge> *> get_edges(
    const std::string & source,
    const std::string & target) = 0;

  virtual std::string to_string() const = 0;

  virtual std::vector<std::string> get_node_names_by_id(const std::string & expr) = 0;
  virtual std::vector<std::string> get_node_names_by_type(const std::string & type) = 0;
  virtual std::vector<Edge> get_edges_from_node(
    const std::string & node_src_id,
    const std::string & type = "") = 0;
  virtual std::vector<Edge> get_edges_from_node_by_data(
    const std::string & node_src_id, const std::string & expr,
    const std::string & type = "") = 0;
  virtual std::vector<Edge> get_edges_by_data(
    const std::string & expr,
    const std::string & type = "") = 0;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH__GRAPHINTERFACE_HPP_
