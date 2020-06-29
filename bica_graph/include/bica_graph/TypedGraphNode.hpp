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

#ifndef BICA_GRAPH__TYPEDGRAPHNODE_HPP_
#define BICA_GRAPH__TYPEDGRAPHNODE_HPP_

#include <boost/optional.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <utility>

#include "bica_graph/Graph.hpp"
#include "bica_graph/GraphNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bica_graph
{

class TypedGraphNode : public GraphNode
{
public:
  explicit TypedGraphNode(const std::string & provided_node_name);

  bool add_tf_edge(TFEdge & tfedge, bool static_tf = false);
  boost::optional<TFEdge> get_tf_edge(const std::string & source, const std::string & target);
  void set_tf_identity(const std::string & frame_id_1, const std::string & frame_id_2);

private:
  void init_tf();

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2::BufferCore> tfBuffer_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH__TYPEDGRAPHNODE_HPP_
