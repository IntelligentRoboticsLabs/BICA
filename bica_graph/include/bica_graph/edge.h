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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


namespace bica_graph
{

typedef enum {UNKNOWN, STRING, DOUBLE, TF} EdgeType;

class EdgeBase : public std::enable_shared_from_this<EdgeBase>
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(EdgeBase)

  EdgeBase() {}

  virtual ~EdgeBase() {}

  template<class T> const T get() const;
  template<class T, class U> void set(const U& rhs);

  EdgeType get_type() const {return type_;}

  friend bool operator==(const EdgeBase& lhs,const EdgeBase& rhs);

  const std::string get_source() const {return source_;}
  const std::string get_target() const {return target_;}

protected:
  EdgeType type_;

  std::string source_;
  std::string target_;
};

template <class T>
EdgeType to_type()
{
  if (std::is_same<T, std::string>::value)
    return STRING;
  else if (std::is_same<T, double>::value)
    return DOUBLE;
  else if (std::is_same<T, tf::Transform>::value)
    return TF;
  else
    return UNKNOWN;
}

template<class T>
class Edge: public EdgeBase
{
public:
  Edge(const std::string& source, const std::string& target, const T& data)
  : EdgeBase(), data_(data)
  {
    type_ = to_type<T>();

    source_ = source;
    target_ = target;
  }

  Edge(const std::string& source, const std::string& target)
  : Edge(source, target, T())
  {}

  const T get() const {return data_;}
  void set(const T& rhs) {data_=rhs;}

private:
  T data_;
};

class BicaTransformListener: public Singleton<tf::TransformListener> {};
class BicaTransformBroadcaster: public Singleton<tf::TransformBroadcaster> {};

template<>
class Edge<tf::Transform>: public EdgeBase
{
public:
  Edge(const std::string& source, const std::string& target, const tf::Transform& data)
  : EdgeBase(), nh_()
  {
    tf_listener_ = BicaTransformListener::getInstance();
    tf_broadcaster_ = BicaTransformBroadcaster::getInstance();

    type_ = to_type<tf::Transform>();

    source_ = source;
    target_ = target;

    publish_transform(source_, target_, data);
  }

  Edge(const std::string& source, const std::string& target)
  : EdgeBase()
  {
    tf_listener_ = BicaTransformListener::getInstance();
    tf_broadcaster_ = BicaTransformBroadcaster::getInstance();

    type_ = to_type<tf::Transform>();

    source_ = source;
    target_ = target;
  }

  const tf::Transform get() const {

    tf::StampedTransform tf;

    try
    {
      tf_listener_->waitForTransform(source_, target_, ros::Time(0), ros::Duration(1.0));
      tf_listener_->lookupTransform(source_, target_, ros::Time(0), tf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    return tf;
  }

  void set(const tf::Transform& data) {
    publish_transform(source_, target_, data);
  }

  void publish_transform(const std::string& source, const std::string& target, const tf::Transform& data)
  {
    geometry_msgs::TransformStamped tf_send;
    tf_send.child_frame_id = target_;
    tf_send.header.frame_id = source_;

    tf_send.header.stamp = ros::Time::now();
    tf::transformTFToMsg(data, tf_send.transform);

    try
    {
      tf_broadcaster_->sendTransform(tf_send);
    }
    catch(tf::TransformException &exception)
    {
      ROS_ERROR("set_transform:: %s", exception.what());
    }
  }

private:
  ros::NodeHandle nh_;

  tf::TransformListener *tf_listener_;
  tf::TransformBroadcaster *tf_broadcaster_;
};

template<class T> const T EdgeBase::get() const
{ return dynamic_cast<const Edge<T>&>(*this).get(); }
template<class T, class U> void EdgeBase::set(const U& rhs)
{ return dynamic_cast<Edge<T>&>(*this).set(rhs); }

bool operator==(const EdgeBase& lhs,const EdgeBase& rhs);

}  // namespace bica_graph
#endif  // BICA_GRAPH_NODE_H
