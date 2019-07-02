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

TFEdge::TFEdge(const std::string& source, const tf::Transform& data, const std::string& target)
{
  source_ = source;
  target_ = target;

  tf_listener_ = BicaTransformListener::getInstance();
  tf_broadcaster_ = BicaTransformBroadcaster::getInstance();

  publish_transform(source_, target_, data);
}

TFEdge::TFEdge(const std::string& source, const std::string& target)
{
  source_ = source;
  target_ = target;

  tf_listener_ = BicaTransformListener::getInstance();
  tf_broadcaster_ = BicaTransformBroadcaster::getInstance();
}

TFEdge::TFEdge(const TFEdge& other)
{
  source_ = other.source_;
  target_ = other.target_;

  tf_listener_ = BicaTransformListener::getInstance();
  tf_broadcaster_ = BicaTransformBroadcaster::getInstance();
}

const tf::Transform
TFEdge::get() const
{
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

void
TFEdge::set(const tf::Transform& data)
{
  std::cerr << "Set Edge (" << get_source() << ")---[(" <<
    data.getOrigin().x() << ", " <<
    data.getOrigin().y() << ", " <<
    data.getOrigin().z() <<
    ")]--->(" << get_target() << ")" << std::endl;
  publish_transform(source_, target_, data);
}

void
TFEdge::publish_transform(const std::string& source, const std::string& target, const tf::Transform& data)
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

  return true;
}

bool operator!=(const TFEdge& lhs, const TFEdge& rhs)
{
  return !(lhs == rhs);
}

}  // namespace bica_graph
