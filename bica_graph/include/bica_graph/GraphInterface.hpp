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

#ifndef BICA_GRAPH_GRAPHINTERFACE__HPP_
#define BICA_GRAPH_GRAPHINTERFACE__HPP_

#include <vector>
#include <map>
#include <string>

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

  virtual std::optional<std::vector<Edge>*> get_edges(
    const std::string & source,
    const std::string & target) = 0;

  virtual std::string to_string() const = 0;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_H
