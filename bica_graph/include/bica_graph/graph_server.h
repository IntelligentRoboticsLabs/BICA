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

#ifndef BICA_GRAPH_GRAPH_SERVER_H
#define BICA_GRAPH_GRAPH_SERVER_H

#include "bica_graph/conversions.h"
#include "bica_msgs/UpdateGraph.h"


namespace bica_graph
{

class GraphServer
{
public:
  GraphServer();
  void publish_graph();

private:

  bool handle_node_update(const bica_msgs::GraphUpdate& update);
  bool handle_string_edge_update(const bica_msgs::GraphUpdate& update);
  bool handle_double_edge_update(const bica_msgs::GraphUpdate& update);
  bool handle_tf_edge_update(const bica_msgs::GraphUpdate& update);

  bool update_service_handler(bica_msgs::UpdateGraph::Request  &req,
                              bica_msgs::UpdateGraph::Response &rep);

  void graph_callback(const bica_msgs::Graph::ConstPtr& msg);
  
protected:
  Graph::SharedPtr graph_;

  ros::NodeHandle nh_;
  ros::Publisher graph_sub_;
  ros::ServiceServer update_srv_server_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_H
