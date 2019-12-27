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


#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <map>
#include <algorithm>
#include <sstream>

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
    while (!already_exist && it != previous_edges.value()->end()) {
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
  } catch (std::exception & e) {
    RCLCPP_WARN(node_->get_logger(),
      "GraphClient::get_tf_edge Nodes [%s, %s]not connected by TFs",
      source.c_str(), target.c_str());
    return {};
  }
}

void
TypedGraphNode::set_tf_identity(const std::string & frame_id_1, const std::string & frame_id_2)
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
