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

#ifndef BICA_GRAPH_EDGE_H
#define BICA_GRAPH_EDGE_H

#include <list>
#include <memory>
#include <string>

#include "bica_graph/macros.h"
#include "bica_graph/Singleton.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>


namespace bica_graph
{

class StringEdge
{
public:
  StringEdge(const std::string& source, const std::string& data, const std::string& target);
  StringEdge(const StringEdge& other);

  const std::string get_source() const {return source_;}
  const std::string get_target() const {return target_;}

  const std::string& get() const {return data_;}
  void set(const std::string& data) {data_ = data;}

  friend bool operator==(const StringEdge& lhs, const StringEdge& rhs);
  friend bool operator!=(const StringEdge& lhs, const StringEdge& rhs);
private:
  std::string source_;
  std::string target_;
  std::string data_;
};

class DoubleEdge
{
public:
  DoubleEdge(const std::string& source, const double data, const std::string& target);
  DoubleEdge(const std::string& source, const std::string& target);
  DoubleEdge(const DoubleEdge& other);

  const std::string get_source() const {return source_;}
  const std::string get_target() const {return target_;}

  const double get() const {return data_;}
  void set(const double& data) {data_ = data;}

  friend bool operator==(const DoubleEdge& lhs, const DoubleEdge& rhs);
  friend bool operator!=(const DoubleEdge& lhs, const DoubleEdge& rhs);
private:
  std::string source_;
  std::string target_;
  double data_;
};


class BicaTransformBuffer: public Singleton<tf2_ros::Buffer> {};
class BicaTransformBroadcaster: public Singleton<tf2_ros::TransformBroadcaster> {};
class BicaStaticTransformBroadcaster: public Singleton<tf2_ros::StaticTransformBroadcaster> {};
class BicaTransformListener: public SingletonRef<tf2_ros::TransformListener, tf2_ros::Buffer>{};

class TFEdge
{
public:
  TFEdge(const std::string& source, const tf2::Transform& data, const std::string& target, bool static_tf = false);
  TFEdge(const std::string& source, const std::string& target, bool static_tf = false);
  TFEdge(const TFEdge& other);

  const std::string get_source() const {return source_;}
  const std::string get_target() const {return target_;}
  bool is_static() const {return static_tf_;}

  const tf2::Transform get() const;
  void set(const tf2::Transform& data);

  friend bool operator==(const TFEdge& lhs, const TFEdge& rhs);
  friend bool operator!=(const TFEdge& lhs, const TFEdge& rhs);

private:
  void publish_transform(const std::string& source, const std::string& target, const tf2::Transform& data);

  ros::NodeHandle nh_;

  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  tf2_ros::TransformListener *tf_listener_;
  tf2_ros::Buffer *tfBuffer;
  tf2_ros::TransformBroadcaster *tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster *static_tf_broadcaster_;

  std::string source_;
  std::string target_;
  std::string data_;

  bool static_tf_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_EDGE_H
