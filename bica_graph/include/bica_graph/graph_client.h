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

#ifndef BICA_GRAPH_GRAPH_CLIENT_H
#define BICA_GRAPH_GRAPH_CLIENT_H

#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "bica_graph/conversions.h"
#include "bica_graph/exceptions.h"
#include "bica_msgs/UpdateGraph.h"

#define TF_EXPIRATION_DURATION  120.0

namespace bica_graph
{

class GraphClient
{
public:

  GraphClient();

  void add_node(const std::string& id, const std::string& type);
  void remove_node(const std::string& id);

  void add_edge(const std::string& source, const std::string& data, const std::string& target);
  void add_edge(const std::string& source, const double& data, const std::string& target);
  void add_edge(const std::string& source, const tf::Transform& data, const std::string& target);

  void add_tf_edge(const std::string& source, const std::string& target);

  Node::SharedPtr get_node(const std::string& id);
  Node::ConstSharedPtr get_const_node(const std::string& id) const;

  size_t count_nodes() const;
  bool exist_node(const std::string& id) const;

  size_t count_edges(const std::string& source, const std::string& target) const;

  const std::map<std::string, Node::SharedPtr>& get_nodes() const;
  const std::map<std::pair<std::string, std::string>, std::list<EdgeBase::SharedPtr>>&
  get_edges() const;

  void print();

  tf::StampedTransform get_tf(const std::string& node_src, const std::string& node_target);
  void set_tf_identity(const std::string& frame_id_1, const std::string& frame_id_2);

  template<class T>
  bool exist_edge(const std::string& source, const std::string& target, const T& data = T()) const
  {
    return graph_->exist_edge<T>(source, target, data);
  }

  template<class T>
  void remove_edge(const std::string& source, const std::string& target, const T& data = T()) {};

  template<class T>
  std::shared_ptr<Edge<T>>
  get_edge(const std::string& source, const std::string& target, const T& data = T())
  {
    return graph_->get_edge<T>(source, target, data);
  }

  template<class T>
  const std::shared_ptr<Edge<T>>
  get_const_edge(const std::string& source, const std::string& target, const T& data = T()) const
  {
    return graph_->get_const_edge<T>(source, target, data);
  }

private:
  void graph_callback(const bica_msgs::Graph::ConstPtr& msg);

protected:
  Graph::SharedPtr graph_;

  ros::NodeHandle nh_;
  ros::Subscriber graph_sub_;
  ros::ServiceClient update_srv_client_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf::TransformListener tf_listener_;
};


template<>
void
GraphClient::remove_edge<double>(const std::string& source, const std::string& target, const double& data)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.edge_source = source;
  srv.request.update.edge_target = target;

  srv.request.update.element_type = bica_msgs::GraphUpdate::DOUBLE_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_edge<double>(source, target);
    else
      ROS_ERROR("Failed to remove edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove node");
  }
}

template<>
void
GraphClient::remove_edge<std::string>(const std::string& source, const std::string& target, const std::string& data)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.edge_source = source;
  srv.request.update.edge_target = target;

  srv.request.update.element_type = bica_msgs::GraphUpdate::STRING_EDGE;
  srv.request.update.edge_type = data;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_edge<std::string>(source, target, data);
    else
      ROS_ERROR("Failed to remove edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove edge");
  }
}

template<>
void
GraphClient::remove_edge<tf::Transform>(const std::string& source, const std::string& target, const tf::Transform& data)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.edge_source = source;
  srv.request.update.edge_target = target;

  srv.request.update.element_type = bica_msgs::GraphUpdate::TF_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_edge<tf::Transform>(source, target);
    else
      ROS_ERROR("Failed to remove edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove edge");
  }
}




}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_H
