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

#ifndef BICA_GRAPH_TYPES__HPP_
#define BICA_GRAPH_TYPES__HPP_

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

bool operator==(const Node & op1, const Node & op2);
bool operator==(const Edge & op1, const Edge & op2);

}  // namespace bica_graph

#endif