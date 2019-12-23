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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <map>
#include <algorithm>
#include <sstream>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "bica_graph/TypedGraphNode.hpp"

namespace bica_graph
{

TypedGraphNode::TypedGraphNode(const std::string & provided_node_name)
: GraphNode(provided_node_name)
{
  static_tf_broadcaster_ = nullptr;
  tfBuffer_ = nullptr;
  tf_listener_ = nullptr;
  tf_broadcaster_ = nullptr;
}

void
TypedGraphNode::init_tf()
{
  tf_broadcaster_ =
    std::make_shared<tf2_ros::TransformBroadcaster>(*node_->shared_from_this());
  static_tf_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node_->shared_from_this());
  tfBuffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, node_->shared_from_this(), false);
}

bool
TypedGraphNode::add_tf_edge(TFEdge & tfedge, bool static_tf)
{
  if (tfBuffer_ == nullptr) {
    init_tf();
  }  
  rclcpp::spin_some(node_);

  tfedge.tf_.header.stamp = node_->now();

  Edge edge;
  edge.type = "tf";
  edge.content = "";
  
  edge.source = tfedge.tf_.header.frame_id;
  edge.target = tfedge.tf_.child_frame_id;

  bool already_exist = false;
  auto previous_edges = get_edges(edge.source, edge.target);
  if (previous_edges.has_value()) {
    auto it = previous_edges.value()->begin();
    while (!already_exist && it != previous_edges.value()->end())
    {
      if (it->type == "tf") {
        already_exist = true;
      }
      it++;
    }
  }

  if (!already_exist) {
    if (GraphNode::add_edge(edge)) {
      if (static_tf) {
        static_tf_broadcaster_->sendTransform(tfedge.tf_);
      } else {
        tf_broadcaster_->sendTransform(tfedge.tf_);
      }
    }
  } else {
      if (static_tf) {
        static_tf_broadcaster_->sendTransform(tfedge.tf_);
      } else {
        tf_broadcaster_->sendTransform(tfedge.tf_);
      }
    return true;
  }
}

std::optional<TFEdge>
TypedGraphNode::get_tf_edge(const std::string & source, const std::string & target)
{
  if (tfBuffer_ == nullptr) {
    init_tf();
  }
  rclcpp::spin_some(node_);

  TFEdge ret;

  geometry_msgs::msg::TransformStamped tf;
  std::string error;

  try {
    tf = tfBuffer_->lookupTransform(source, target, tf2::TimePointZero);
    tf2::convert(tf, ret.tf_);
    return ret;
  } catch(std::exception &e) {
    RCLCPP_WARN(node_->get_logger(),
      "GraphClient::get_tf_edge Nodes [%s, %s]not connected by TFs",
      source.c_str(), target.c_str());
    return {};
  }
}

void 
TypedGraphNode::set_tf_identity(const std::string& frame_id_1, const std::string& frame_id_2)
{
  geometry_msgs::msg::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = node_->now();
  static_transformStamped.header.frame_id = frame_id_1;
  static_transformStamped.child_frame_id = frame_id_2;
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;

  static_tf_broadcaster_->sendTransform(static_transformStamped);
  rclcpp::spin_some(node_);
}


}  // namespace bica_graph
