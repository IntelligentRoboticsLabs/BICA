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
#include <memory>
#include <string>
#include <iostream>

#include <bica_graph/node.h>
#include <bica_graph/exceptions.h>

namespace bica_graph
{
class Node;

class BicaGraph
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(BicaGraph)

  /// Create a new knowledge graph.
  /**
  */
  BicaGraph();

  /// Create a sub knowledge graph. This graph only contains a node and its relations.
  /**
  * \param[in] node The node from which creating a subgraph.
  */
  explicit BicaGraph(const std::shared_ptr<Node>& node);

  /// Count the number of nodes in the grahp.
  /**
  * \returns number of nodes in the graph
  */
  size_t count_nodes() const;

  /// Return if this graph is built from a node graph. In this case, it is locked and no more nodes can be added
  /**
  * \returns true if it is a subgraph
  */
  bool is_sub_graph() const;

  /// Create a node and insert it in the graph.
  /**
  * \param[in] id The id of the new node.
  * \param[in] type The type of the node.
  * \returns true if it is a subgraph
  */
  std::shared_ptr<Node> create_node(const std::string& id, const std::string& type);

  /// Get a node pointer.
  /**
  * \param[in] id The id of the  node.
  * \returns The shared pointer to the node
  */
  std::shared_ptr<Node> get_node(const std::string& id);


  /// Get the list of nodes of in the graph.
  /**
  * \returns the list of nodes as a const reference
  */
  const std::list<std::shared_ptr<Node>>& get_nodes() const {return nodes_;}

  /// Compare tho graphs.
  /**
  * \param[in] other The graph to compare.
  * \returns true if both graphs are equals
  */
  friend bool operator==(const BicaGraph& lhs, const BicaGraph& rhs);

  /// Fill a text stream for printing graphs.
  /**
  * \param[in] lhs The stream to print to.
  * \param[in] other The graph to stream.
  * \returns the ostream
  */
  friend std::ostream& operator<<(std::ostream& lhs, const BicaGraph& rhs);

private:
  std::list<std::shared_ptr<Node>> nodes_;
  bool locked_;
};


bool operator==(const BicaGraph& lhs, const BicaGraph& rhs);
std::ostream& operator<<(std::ostream& lhs, const BicaGraph& rhs);

}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_H
