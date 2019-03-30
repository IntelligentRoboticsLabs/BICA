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

#include <bica_graph/graph_handler.h>
#include <bica_graph/conversions.h>

#include <string>

using bica_graph::GraphHandler;

GraphHandler::GraphHandler(ros::NodeHandle& nh, std::string handler_id)
: nh_(nh),
  graph_(new bica_graph::BicaGraph()),
  handler_id_(handler_id)
{
  graph_update_pub_ = nh_.advertise<bica_msgs::GraphUpdate>("/graph_updates", 1000);
  graph_update_sub_ = nh_.subscribe("/graph_updates", 1000, &GraphHandler::graph_update_callback, this);
  graph_pub_ = nh_.advertise<bica_msgs::Graph>("graph", 1, true);
  graph_timer_ = nh_.createTimer(ros::Duration(1.0),  &GraphHandler::timer_callback, this);
}

void
GraphHandler::graph_update_callback(const ros::MessageEvent<bica_msgs::GraphUpdate const>& event)
{
  if ((event.getPublisherName() == ros::this_node::getName()) && (handler_id_ == event.getMessage()->handler_id))
    return;

  const bica_msgs::GraphUpdateConstPtr& msg = event.getMessage();

  switch (msg->element_type)
  {
    case bica_msgs::GraphUpdate::NODE:
      update_node(msg);
      break;
    case bica_msgs::GraphUpdate::RELATION:
      update_relation(msg);
      break;
    case bica_msgs::GraphUpdate::TF_RELATION:
      update_tf_relation(msg);
      break;
    default:
      ROS_ERROR("GraphHandler::graph_update_callback Message not recognized");
  }
}

void
GraphHandler::timer_callback(const ros::TimerEvent& event)
{
  bica_msgs::Graph::ConstPtr msg = bica_graph::graph_to_msg(*graph_);

  graph_pub_.publish(msg);
}

void
GraphHandler::update_node(const bica_msgs::GraphUpdateConstPtr& msg)
{
  switch (msg->update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      {
        auto node = graph_->get_node(msg->node_id);

        if (node == nullptr)
        {
          auto new_node = graph_->create_node(msg->node_id, msg->node_type, msg->stamp);
        }
        else
        {
          if (node->get_type() != msg->node_type)
          {
            ROS_ERROR("GraphHandler::update_node Type mismatch for %s (%s != %s)",
              node->get_id().c_str(), node->get_type().c_str(), msg->node_type.c_str());
          }
          else
            auto updated_node = graph_->create_node(msg->node_id, msg->node_type);
        }
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      {
        auto node = graph_->get_node(msg->node_id);

        if (node != nullptr)
        {
          if (node->get_time_stamp() < msg->stamp)
            graph_->remove_node(msg->node_id);
        }
      }
      break;
    default:
      ROS_ERROR("GraphHandler::update_node Operation not recognized");
  }
}

void
GraphHandler::update_relation(const bica_msgs::GraphUpdateConstPtr& msg)
{
  auto node_source = graph_->get_node(msg->relation_source);
  auto node_target = graph_->get_node(msg->relation_target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::update_relation Node source (%s) not found",
      msg->relation_source.c_str());
    return;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::update_relation Node target (%s) not found",
      msg->relation_target.c_str());
    return;
  }

  switch (msg->update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      {
        auto new_relation = node_source->add_relation(msg->relation_type, node_target);
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      {
        auto relation = node_source->get_relation(node_target, msg->relation_type);

        if (relation != nullptr && relation->get_time_stamp() < msg->stamp)
          node_source->remove_relation(msg->relation_type, node_target);
      }
      break;
    default:
      ROS_ERROR("GraphHandler::update_relation Operation not recognized");
  }
}

void
GraphHandler::update_tf_relation(const bica_msgs::GraphUpdateConstPtr& msg)
{
  auto node_source = graph_->get_node(msg->relation_source);
  auto node_target = graph_->get_node(msg->relation_target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::update_tf_relation Node source (%s) not found",
      msg->relation_source.c_str());
    return;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::update_tf_relation Node target (%s) not found",
      msg->relation_target.c_str());
    return;
  }

  switch (msg->update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      {
        auto new_relation = node_source->add_tf_relation(node_target);
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      {
        auto relation = node_source->get_tf_relation(node_target);

        if (relation != nullptr && relation->get_time_stamp() < msg->stamp)
          node_source->remove_tf_relation(node_target);
      }
      break;
    default:
      ROS_ERROR("GraphHandler::update_tf_relation Operation not recognized");
  }
}

std::shared_ptr<bica_graph::Node>
GraphHandler::create_node(const std::string& id, const std::string& type)
{
  auto new_node = graph_->create_node(id, type);

  bica_msgs::GraphUpdate msg;

  msg.stamp = new_node->get_time_stamp();
  msg.handler_id = handler_id_;
  msg.element_type = bica_msgs::GraphUpdate::NODE;
  msg.update_type = bica_msgs::GraphUpdate::ADD;
  msg.node_id = id;
  msg.node_type = type;

  graph_update_pub_.publish(msg);

  return new_node;
}

void
GraphHandler::remove_node(const std::string& id)
{
  graph_->remove_node(id);

  bica_msgs::GraphUpdate msg;

  msg.stamp = ros::Time::now();
  msg.handler_id = handler_id_;
  msg.element_type = bica_msgs::GraphUpdate::NODE;
  msg.update_type = bica_msgs::GraphUpdate::REMOVE;
  msg.node_id = id;

  graph_update_pub_.publish(msg);
}

std::shared_ptr<bica_graph::Node>
GraphHandler::get_node(const std::string& id) const
{
  return graph_->get_node(id);
}

size_t
GraphHandler::count_nodes() const
{
  return graph_->count_nodes();
}

std::shared_ptr<bica_graph::Relation>
GraphHandler::add_relation(const std::string& source,
  const std::string& type,
  const std::string& target)
{
  auto node_source = graph_->get_node(source);
  auto node_target = graph_->get_node(target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::add_relation Node source (%s) not found",
      source.c_str());
    return nullptr;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::add_relation Node target (%s) not found",
      target.c_str());
    return nullptr;
  }

  auto relation = node_source->add_relation(type, node_target);

  bica_msgs::GraphUpdate msg;

  msg.stamp = relation->get_time_stamp();
  msg.handler_id = handler_id_;
  msg.element_type = bica_msgs::GraphUpdate::RELATION;
  msg.update_type = bica_msgs::GraphUpdate::ADD;
  msg.relation_source = source;
  msg.relation_target = target;
  msg.relation_type = type;

  graph_update_pub_.publish(msg);

  return relation;
}

std::shared_ptr<bica_graph::Relation>
GraphHandler::add_tf_relation(const std::string& source,
  const tf::Transform& tf,
  const std::string& target)
{
  auto node_source = graph_->get_node(source);
  auto node_target = graph_->get_node(target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::add_tf_relation Node source (%s) not found",
      source.c_str());
    return nullptr;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::add_tf_relation Node target (%s) not found",
      target.c_str());
    return nullptr;
  }

  auto tf_relation = node_source->add_tf_relation(tf, node_target);

  bica_msgs::GraphUpdate msg;

  msg.stamp = tf_relation->get_time_stamp();
  msg.handler_id = handler_id_;
  msg.element_type = bica_msgs::GraphUpdate::TF_RELATION;
  msg.update_type = bica_msgs::GraphUpdate::ADD;
  msg.relation_source = source;
  msg.relation_target = target;
  msg.relation_type = "tf";

  graph_update_pub_.publish(msg);

  return tf_relation;
}

std::shared_ptr<bica_graph::TFRelation>
GraphHandler::get_tf_relation(const std::string& source,
  const std::string& target)
{
  auto node_source = graph_->get_node(source);
  auto node_target = graph_->get_node(target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::get_tf_relation Node source (%s) not found",
      source.c_str());
    return nullptr;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::get_tf_relation Node target (%s) not found",
      target.c_str());
    return nullptr;
  }

  return node_source->get_tf_relation(node_target);
}

void
GraphHandler::remove_relation(const std::string& source,
  const std::string& type,
  const std::string& target)
{
  auto node_source = graph_->get_node(source);
  auto node_target = graph_->get_node(target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::remove_relation Node source (%s) not found",
      source.c_str());
    return;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::remove_relation Node target (%s) not found",
      target.c_str());
    return;
  }

  if (node_source->get_relation(node_target, type) != nullptr)
  {
    node_source->remove_relation(type, node_target);

    bica_msgs::GraphUpdate msg;

    msg.stamp = ros::Time::now();
    msg.handler_id = handler_id_;
    msg.element_type = bica_msgs::GraphUpdate::RELATION;
    msg.update_type = bica_msgs::GraphUpdate::REMOVE;
    msg.relation_source = source;
    msg.relation_target = target;
    msg.relation_type = type;

    graph_update_pub_.publish(msg);
  }
}

void
GraphHandler::remove_tf_relation(const std::string& source,
  const std::string& target)
{
  auto node_source = graph_->get_node(source);
  auto node_target = graph_->get_node(target);

  if (node_source == nullptr)
  {
    ROS_ERROR("GraphHandler::remove_relation Node source (%s) not found",
      source.c_str());
    return;
  }

  if (node_target == nullptr)
  {
    ROS_ERROR("GraphHandler::remove_relation Node target (%s) not found",
      target.c_str());
    return;
  }

  if (node_source->get_tf_relation(node_target) != nullptr)
  {
    node_source->remove_tf_relation(node_target);

    bica_msgs::GraphUpdate msg;

    msg.stamp = ros::Time::now();
    msg.handler_id = handler_id_;
    msg.element_type = bica_msgs::GraphUpdate::TF_RELATION;
    msg.update_type = bica_msgs::GraphUpdate::REMOVE;
    msg.relation_source = source;
    msg.relation_target = target;
    msg.relation_type = "tf";

    graph_update_pub_.publish(msg);
  }
}

bool
GraphHandler::contains_node(const std::string& node_id)
{
  return get_node(node_id) != nullptr;
}

bool
GraphHandler::contains_relation(const std::string& source,
  const std::string& type,
  const std::string& target)
{
  auto node_source = get_node(source);
  auto node_target = get_node(target);

  if (node_source == nullptr)
    return false;

  if (node_target == nullptr)
    return false;

  return node_source->get_relation(node_target, type) != nullptr;
}

bool
GraphHandler::contains_tf_relation(const std::string& source,
  const std::string& target)
{
  return contains_relation(source, "tf", target);
}
