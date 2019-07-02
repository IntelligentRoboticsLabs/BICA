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

#ifndef BICA_GRAPH_CONVERSIONS_H
#define BICA_GRAPH_CONVERSIONS_H
#include <string>

#include "bica_msgs/Graph.h"
#include "bica_graph/graph.h"
#include "bica_graph/exceptions.h"

namespace bica_graph
{
  /// Convert a Node to a bica_msgs::Node message.
  /**
  * \param[in] node A shared pointer reference to the node to create
  * \param[out] msg A pointer to a pre-existing bica_msgs::Node
  */
  void node_to_msg(const Node& node, bica_msgs::Node* msg);

  /// Convert a bica_graph::Graph to a bica_msgs::Graph message.
  /**
  * \param[in] graph The graph
  * \returns the pointer of a new created message
  */
  void graph_to_msg(const bica_graph::Graph& graph, bica_msgs::Graph* msg);

  /// Convert a bica_graph::Graph to a bica_msgs::Graph message.
  /**
  * \param[in] graph The graph
  * \returns the pointer of a new created message
  */
  void msg_to_graph(const bica_msgs::Graph& msg, bica_graph::Graph::SharedPtr& graph);

  void edge_to_msg(const bica_graph::StringEdge& edge, bica_msgs::Edge* msg);
  void edge_to_msg(const bica_graph::DoubleEdge& edge, bica_msgs::Edge* msg);
  void edge_to_msg(const bica_graph::TFEdge& edge, bica_msgs::Edge* msg);

}  // namespace bica_graph

#endif  // BICA_GRAPH_CONVERSIONS_H
