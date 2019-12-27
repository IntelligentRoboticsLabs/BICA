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

#ifndef BICA_GRAPH__TYPES_HPP_
#define BICA_GRAPH__TYPES_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <assert.h>
#include <string>
#include <vector>

namespace bica_graph
{

std::vector<std::string> tokenize(const std::string & text, const std::string & delim);

struct Node
{
  std::string name;
  std::string type;

  std::string to_string() const
  {
    return "node::" + name + "::" + type;
  }

  void from_string(const std::string & node_str)
  {
    auto tokens = tokenize(node_str, "::");
    assert(tokens.size() == 3);
    name = tokens[1];
    type = tokens[2];
  }
};

class Edge
{
public:
  std::string content;
  std::string type;

  std::string source;
  std::string target;

  std::string to_string() const
  {
    return "edge::" + source + "->" + target + "::" + content + "::" + type;
  }

  void from_string(const std::string & edge_str)
  {
    auto tokens = tokenize(edge_str, "::");
    assert(tokens.size() == 4);
    auto conn_tokens = tokenize(tokens[1], "->");
    assert(conn_tokens.size() == 2);

    source = conn_tokens[0];
    target = conn_tokens[1];

    content = tokens[2];
    type = tokens[3];
  }
};

class TFEdge
{
public:
  geometry_msgs::msg::TransformStamped tf_;
};

bool operator==(const Node & op1, const Node & op2);
bool operator==(const Edge & op1, const Edge & op2);

}  // namespace bica_graph

#endif  // BICA_GRAPH__TYPES_HPP_
