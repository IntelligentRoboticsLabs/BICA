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

#ifndef BICA_GRAPH_GRAPH_LISTENER_H
#define BICA_GRAPH_GRAPH_LISTENER_H

#include <ros/ros.h>

#include <bica_graph/graph.h>
#include <bica_msgs/Graph.h>

namespace bica_graph
{

class GraphListener
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(GraphListener)

  /// Create a sub knowledge graph. This graph only contains a node and its relations.
  /**
  * \param[in] nh A nodehandle.
  * \param[in] graph The graph to update.
  */
  explicit GraphListener(ros::NodeHandle& nh, const bica_graph::BicaGraph::SharedPtr& graph);

private:
  /// The callback for incoming graph messages.
  /**
  * \param[in] msg The incoming graph message
  */
  void graph_callback(const bica_msgs::Graph::ConstPtr& msg);

  ros::NodeHandle nh_;
  BicaGraph::SharedPtr graph_;

  ros::Subscriber graph_sub_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_LISTENER_H
