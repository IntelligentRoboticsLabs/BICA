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

#include <utility>
#include <map>
#include <string>
#include <list>

#include "bica_graph/graph_client.h"

namespace bica_graph
{

GraphClient::GraphClient()
: tf_listener_(ros::Duration(TF_EXPIRATION_DURATION))
{
  graph_ = std::make_shared<Graph>();

  graph_sub_ = nh_.subscribe("/graph", 1, &GraphClient::graph_callback, this);
  update_srv_client_ = nh_.serviceClient<bica_msgs::UpdateGraph>("/update_graph");

  while (!ros::service::waitForService("/update_graph", 1000))
    ROS_WARN("Waiting for graph master service");
}

void
GraphClient::graph_callback(const bica_msgs::Graph::ConstPtr& msg)
{
  msg_to_graph(*msg, graph_);
}

bool
GraphClient::exist_node(const std::string& id) const
{
  return graph_->exist_node(id);
}

void
GraphClient::add_node(const Node& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.element_type = bica_msgs::GraphUpdate::NODE;
  srv.request.update.update_type = bica_msgs::GraphUpdate::ADD;
  srv.request.update.node_id = other.get_id();
  srv.request.update.node_type = other.get_type();

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->add_node(other);
    else
      ROS_ERROR("Failed to add node");
  }
  else
  {
    ROS_ERROR("Failed to call service to add node");
  }
}

void
GraphClient::add_node(const std::string& id, const std::string& type)
{
  add_node(Node(id, type));
}

const Node&
GraphClient::get_node(const std::string& id)
{
  return graph_->get_node(id);
}

void
GraphClient::remove_node(const std::string& node_id)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.element_type = bica_msgs::GraphUpdate::NODE;
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.node_id = node_id;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_node(node_id);
    else
      ROS_ERROR("Failed to remove node");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove node");
  }
}

size_t
GraphClient::count_nodes() const
{
  return graph_->count_nodes();
}

void
GraphClient::add_edge(const std::string& source, const std::string& data, const std::string& target)
{
  add_edge(StringEdge(source, data, target));
}

void
GraphClient::add_edge(const std::string& source, const double data, const std::string& target)
{
  add_edge(DoubleEdge(source, data, target));
}

void
GraphClient::add_edge(const std::string& source, const tf::Transform& data, const std::string& target)
{
  add_edge(TFEdge(source, data, target));
}

void
GraphClient::add_edge(const StringEdge& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::ADD;
  srv.request.update.edge_source = other.get_source();
  srv.request.update.edge_target = other.get_target();
  srv.request.update.edge_type = other.get();
  srv.request.update.element_type = bica_msgs::GraphUpdate::STRING_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
    {
      graph_->add_edge(other);
    }
    else
      ROS_ERROR("Failed to add edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to add node");
  }
}
void
GraphClient::add_edge(const DoubleEdge& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::ADD;
  srv.request.update.edge_source = other.get_source();
  srv.request.update.edge_target = other.get_target();
  srv.request.update.edge_double = other.get();
  srv.request.update.element_type = bica_msgs::GraphUpdate::DOUBLE_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->add_edge(other);
    else
      ROS_ERROR("Failed to add edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to add node");
  }
}

void
GraphClient::add_edge(const TFEdge& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::ADD;
  srv.request.update.edge_source = other.get_source();
  srv.request.update.edge_target = other.get_target();
  srv.request.update.element_type = bica_msgs::GraphUpdate::TF_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->add_edge(other);
    else
      ROS_ERROR("Failed to add edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to add node");
  }
}

void
GraphClient::add_tf_edge(const std::string& source, const std::string& target)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::ADD;
  srv.request.update.edge_source = source;
  srv.request.update.edge_target = target;
  srv.request.update.element_type = bica_msgs::GraphUpdate::TF_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->add_tf_edge(source, target);
    else
      ROS_ERROR("Failed to add edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to add node");
  }
}

bool
GraphClient::exist_edge(const std::string& source, const std::string& data, const std::string& target)
{
  return graph_->exist_edge(source, data, target);
}

bool
GraphClient::exist_edge(const StringEdge& other)
{
  return graph_->exist_edge(other);
}

bool
GraphClient::exist_edge(const DoubleEdge& other)
{
  return graph_->exist_edge(other);
}

bool
GraphClient::exist_edge(const TFEdge& other)
{
  return graph_->exist_edge(other);
}

bool
GraphClient::exist_tf_edge(const std::string& source, const std::string& target)
{
  return graph_->exist_tf_edge(source, target);
}

bool
GraphClient::exist_double_edge(const std::string& source, const std::string& target)
{
  return graph_->exist_double_edge(source, target);
}

void
GraphClient::remove_edge(const std::string& source, const std::string& data, const std::string& target)
{
  remove_edge(StringEdge(source, data, target));
}

void
GraphClient::remove_edge(const StringEdge& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.edge_source = other.get_source();
  srv.request.update.edge_target = other.get_target();

  srv.request.update.element_type = bica_msgs::GraphUpdate::STRING_EDGE;
  srv.request.update.edge_type = other.get();

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_edge(other);
    else
      ROS_ERROR("Failed to remove edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove edge");
  }
}

void
GraphClient::remove_edge(const DoubleEdge& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.edge_source = other.get_source();
  srv.request.update.edge_target = other.get_target();

  srv.request.update.element_type = bica_msgs::GraphUpdate::DOUBLE_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_edge(other);
    else
      ROS_ERROR("Failed to remove edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove node");
  }
}

void
GraphClient::remove_edge(const TFEdge& other)
{
  bica_msgs::UpdateGraph srv;
  srv.request.update.stamp = ros::Time::now();
  srv.request.update.update_type = bica_msgs::GraphUpdate::REMOVE;
  srv.request.update.edge_source = other.get_source();
  srv.request.update.edge_target = other.get_target();

  srv.request.update.element_type = bica_msgs::GraphUpdate::TF_EDGE;

  if (update_srv_client_.call(srv))
  {
    if (srv.response.success)
      graph_->remove_edge(other);
    else
      ROS_ERROR("Failed to remove edge");
  }
  else
  {
    ROS_ERROR("Failed to call service to remove edge");
  }
}

void
GraphClient::remove_tf_edge(const std::string& source, const std::string& target)
{
  remove_edge(TFEdge(source, target));
}

void
GraphClient::remove_double_edge(const std::string& source, const std::string& target)
{
  remove_edge(DoubleEdge(source, target));
}

DoubleEdge&
GraphClient::get_double_edge(const std::string& source, const std::string& target)
{
  return graph_->get_double_edge(source, target);
}

TFEdge&
GraphClient::get_tf_edge(const std::string& source, const std::string& target)
{
  return graph_->get_tf_edge(source, target);
}

const std::list<Node>&
GraphClient::get_nodes() const
{
  return graph_->get_nodes();
}

const std::list<StringEdge>&
GraphClient::get_string_edges() const
{
  return graph_->get_string_edges();
}

const std::list<DoubleEdge>&
GraphClient::get_double_edges() const
{
  return graph_->get_double_edges();
}

const std::list<TFEdge>&
GraphClient::get_tf_edges() const
{
  return graph_->get_tf_edges();
}


tf::StampedTransform
GraphClient::get_tf(const std::string& node_src, const std::string& node_target)
{
  if (!exist_node(node_src))
    throw exceptions::TransformNotPossible("Source node do not exists");
  if (!exist_node(node_target))
    throw exceptions::TransformNotPossible("Target node do not exists");

  tf::StampedTransform tf;

  try
  {
    tf_listener_.waitForTransform(node_src, node_target, ros::Time(0), ros::Duration(0.1));
    tf_listener_.lookupTransform(node_src, node_target, ros::Time(0), tf);
  }
  catch (tf::TransformException& ex)
  {
    throw exceptions::TransformNotPossible("Nodes not connected");
  }

  return tf;
}

void
GraphClient::set_tf_identity(const std::string& frame_id_1, const std::string& frame_id_2)
{
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = frame_id_1;
  static_transformStamped.child_frame_id = frame_id_2;
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;

  static_tf_broadcaster_.sendTransform(static_transformStamped);
}

void
GraphClient::print()
{
  graph_->print();
}

}  // namespace bica_graph
