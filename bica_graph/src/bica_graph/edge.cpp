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

#include <string>

#include "bica_graph/edge.h"
#include "bica_graph/exceptions.h"

namespace bica_graph
{

StringEdge::StringEdge(const std::string& source, const std::string& data, const std::string& target)
{
  source_ = source;
  target_ = target;
  data_ = data;
}

StringEdge::StringEdge(const StringEdge& other)
{
  source_ = other.source_;
  target_ = other.target_;
  data_ = other.data_;
}

DoubleEdge::DoubleEdge(const std::string& source, const double data, const std::string& target)
{
  source_ = source;
  target_ = target;
  data_ = data;
}

DoubleEdge::DoubleEdge(const std::string& source, const std::string& target)
{
  source_ = source;
  target_ = target;
  data_ = 0.0;
}

DoubleEdge::DoubleEdge(const DoubleEdge& other)
{
  source_ = other.source_;
  target_ = other.target_;
  data_ = other.data_;
}


TFEdge::TFEdge(const std::string& source, const tf2::Transform& data, const std::string& target, bool static_tf)
{
  source_ = source;
  target_ = target;
  static_tf_ = static_tf;

  tf_broadcaster_ = BicaTransformBroadcaster::getInstance();
  static_tf_broadcaster_ = BicaStaticTransformBroadcaster::getInstance();
  tfBuffer = BicaTransformBuffer::getInstance();
  tf_listener_ = BicaTransformListener::getInstance(*tfBuffer);
  //std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  publish_transform(source_, target_, data);
}

TFEdge::TFEdge(const std::string& source, const std::string& target, bool static_tf)
{
  source_ = source;
  target_ = target;
  static_tf_ = static_tf;

  tf_broadcaster_ = BicaTransformBroadcaster::getInstance();
  static_tf_broadcaster_ = BicaStaticTransformBroadcaster::getInstance();
  tfBuffer = BicaTransformBuffer::getInstance();
  tf_listener_ = BicaTransformListener::getInstance(*tfBuffer);
}

TFEdge::TFEdge(const TFEdge& other)
{
  source_ = other.source_;
  target_ = other.target_;
  static_tf_ = other.static_tf_;

  tf_broadcaster_ = BicaTransformBroadcaster::getInstance();
  static_tf_broadcaster_ = BicaStaticTransformBroadcaster::getInstance();
  tfBuffer = BicaTransformBuffer::getInstance();
  tf_listener_ = BicaTransformListener::getInstance(*tfBuffer);
}

const tf2::Transform
TFEdge::get() const
{
  tf2::Stamped<tf2::Transform> ret;
  geometry_msgs::TransformStamped tf;

  std::string error;
  if (tfBuffer->canTransform(source_, target_, ros::Time(0), ros::Duration(0.1), &error))
      tf = tfBuffer->lookupTransform(source_, target_, ros::Time(0));
  else
    ROS_ERROR("Can't transform %s", error.c_str());
    ROS_INFO("(%lf, %lf, %lf)",
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
  tf2::fromMsg(tf, ret);

  return ret;
}

void
TFEdge::set(const tf2::Transform& data)
{
  if (static_tf_)
    throw exceptions::TransformError("static transform cannot be updated");

  publish_transform(source_, target_, data);
}

void
TFEdge::publish_transform(const std::string& source, const std::string& target, const tf2::Transform& data)
{
  if (!static_tf_)
  {
    tf2::Stamped<tf2::Transform> data_stamped;
    data_stamped.frame_id_ = source;
    data_stamped.stamp_ = ros::Time::now();;

    data_stamped.setData(data);

    geometry_msgs::TransformStamped tf_send = tf2::toMsg(data_stamped);
    tf_send.child_frame_id = target;

    ROS_INFO("(%lf, %lf, %lf)",
      tf_send.transform.translation.x,
      tf_send.transform.translation.y,
      tf_send.transform.translation.z);

    try
    {
      tf_broadcaster_->sendTransform(tf_send);
    }
    catch(tf2::TransformException &exception)
    {
      ROS_ERROR("set_transform:: %s", exception.what());
    }
  }
  else
  {
    tf2::Stamped<tf2::Transform> data_stamped;
    data_stamped.frame_id_ = source;
    data_stamped.stamp_ = ros::Time::now();

    data_stamped.setData(data);

    geometry_msgs::TransformStamped tf_send = tf2::toMsg(data_stamped);
    tf_send.child_frame_id = target_;

    static_tf_broadcaster_->sendTransform(tf_send);
  }
}

bool operator==(const StringEdge& lhs, const StringEdge& rhs)
{
  if (lhs.source_ != rhs.source_) return false;
  if (lhs.target_ != rhs.target_) return false;
  if (lhs.data_ != rhs.data_) return false;

  return true;
}

bool operator!=(const StringEdge& lhs, const StringEdge& rhs)
{
  return !(lhs == rhs);
}

bool operator==(const DoubleEdge& lhs, const DoubleEdge& rhs)
{
  if (lhs.source_ != rhs.source_) return false;
  if (lhs.target_ != rhs.target_) return false;

  return true;
}

bool operator!=(const DoubleEdge& lhs, const DoubleEdge& rhs)
{
  return !(lhs == rhs);
}

bool operator==(const TFEdge& lhs, const TFEdge& rhs)
{
  if (lhs.source_ != rhs.source_) return false;
  if (lhs.target_ != rhs.target_) return false;
  // if (lhs.static_tf_ != rhs.static_tf_) return false;

  return true;
}

bool operator!=(const TFEdge& lhs, const TFEdge& rhs)
{
  return !(lhs == rhs);
}

}  // namespace bica_graph
