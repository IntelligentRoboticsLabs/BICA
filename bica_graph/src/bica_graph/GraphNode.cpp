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

#include "bica_graph/GraphNode.hpp"

#include "bica_msgs/msg/graph_update.hpp"


namespace bica_graph
{

GraphNode::GraphNode(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node), seq_(0)
{
  last_sub_count_ = node_->count_publishers("/graph_updates");
  initialized_ = last_sub_count_ == 0;
  last_ts_ = node_->now();

  using namespace std::placeholders;
  update_pub_ = node_->create_publisher<bica_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(20).keep_last(20).reliable().transient_local());
  
  update_sub_ = node_->create_subscription<bica_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(20).keep_last(20).reliable().transient_local(),
    std::bind(&GraphNode::update_callback, this, _1));
}

void
GraphNode::update_callback(const bica_msgs::msg::GraphUpdate::SharedPtr msg)
{
  updates_.push_back(*msg);
}

bool sort_function(const bica_msgs::msg::GraphUpdate & op1, const bica_msgs::msg::GraphUpdate & op2)
{ 
  return (rclcpp::Time(op1.stamp) < rclcpp::Time(op2.stamp)); 
}

void
GraphNode::process_updates()
{
  bool updated_graph = false;
  std::sort (updates_.begin(), updates_.end(), sort_function);  

  for (const auto & update : updates_) {
    last_ts_ = rclcpp::Time(update.stamp);
    updated_graph = true;
    
    if (update.element_type == bica_msgs::msg::GraphUpdate::NODE) {
      Node node;
      node.from_string(update.object);

      if (update.operation_type == bica_msgs::msg::GraphUpdate::ADD) {
        graph_.add_node(node);
        // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " << update.node_id << "]\t" << seq_ << "\tADD " <<node.to_string() <<std::endl;
      } else if (update.operation_type == bica_msgs::msg::GraphUpdate::REMOVE) {
        graph_.remove_node(node.name);
        // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " << update.node_id << "]\t" << seq_ << "\tREMOVE " <<node.to_string() <<std::endl;
      }
      seq_++;
    } else if (update.element_type == bica_msgs::msg::GraphUpdate::EDGE) {
      Edge edge;
      edge.from_string(update.object);
    
      if (update.operation_type == bica_msgs::msg::GraphUpdate::ADD) {
        graph_.add_edge(edge);
        // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " << update.node_id << "]\t" << seq_ << "\tADD " << edge.to_string() << std::endl;
      } else if (update.operation_type == bica_msgs::msg::GraphUpdate::REMOVE) {
        graph_.remove_edge(edge);
        // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " << update.node_id << "]\t" << seq_ << "\tREMOVE " <<edge.to_string() <<std::endl;
      }
      seq_++;
    } else if (update.element_type == bica_msgs::msg::GraphUpdate::GRAPH) {
      if (!initialized_) {
        graph_.from_string(update.object);
        seq_ = update.seq;
        // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " << update.node_id << "]\t" << seq_ << "<\tSYNC" <<std::endl;
        initialized_ = true;
        pending_updates_.clear();
      }
    }
  }
 
  if (updated_graph) {
    chached_graph_.from_string(graph_.to_string());
  } else {
    last_ts_ = node_->now();
  }

  int current_sub_count = node_->count_publishers("/graph_updates");
  if (initialized_ && current_sub_count > 1 && current_sub_count > last_sub_count_) {
    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = last_ts_ + rclcpp::Duration(0.0, 1.0);  // Dt
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::SYNC;
    msg.element_type = bica_msgs::msg::GraphUpdate::GRAPH;
    msg.object =  graph_.to_string();
    msg.seq = seq_;
    update_pub_->publish(msg);
  }
  last_sub_count_ = current_sub_count;

  if (initialized_) {
    for (const auto & pending_msg : pending_updates_) {
      update_pub_->publish(pending_msg);
    }
    pending_updates_.clear();
  }
  
  // std::cout << "==================================================" << std::endl;
  updates_.clear();
}

void
GraphNode::add_node(const Node & node)
{
  chached_graph_.add_node(node);

  bica_msgs::msg::GraphUpdate msg;
  msg.stamp = node_->now();
  msg.node_id = node_->get_name();
  msg.operation_type = bica_msgs::msg::GraphUpdate::ADD;
  msg.element_type = bica_msgs::msg::GraphUpdate::NODE;
  msg.object = node.to_string();

  if (!initialized_) {
    pending_updates_.push_back(msg);
  } else {
    update_pub_->publish(msg);
  }
}

void
GraphNode::remove_node(const std::string node)
{
  chached_graph_.remove_node(node);

  bica_msgs::msg::GraphUpdate msg;
  msg.stamp = node_->now();
  msg.node_id = node_->get_name();
  msg.operation_type = bica_msgs::msg::GraphUpdate::REMOVE;
  msg.element_type = bica_msgs::msg::GraphUpdate::NODE;
  msg.object = Node{node, "no_type"}.to_string();

  if (!initialized_) {
    pending_updates_.push_back(msg);
  } else {
    update_pub_->publish(msg);
  }
}

bool 
GraphNode::exist_node(const std::string node)
{
  return chached_graph_.exist_node(node);
}

std::optional<Node>
GraphNode::get_node(const std::string node)
{
  return chached_graph_.get_node(node);
}

bool
GraphNode::add_edge(const Edge & edge)
{
  if (chached_graph_.exist_node(edge.source) && chached_graph_.exist_node(edge.target) && !chached_graph_.exist_edge(edge)) {
    chached_graph_.add_edge(edge);
    
    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = node_->now();
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::ADD;
    msg.element_type = bica_msgs::msg::GraphUpdate::EDGE;
    msg.object = edge.to_string();

    if (!initialized_) {
      pending_updates_.push_back(msg);
    } else {
      update_pub_->publish(msg);
    }
    
    return true;
  } else {
    return false;
  }
}

bool
GraphNode::remove_edge(const Edge & edge)
{
  if (chached_graph_.exist_node(edge.source) && chached_graph_.exist_node(edge.target) && chached_graph_.exist_edge(edge)) {
    auto edges = chached_graph_.get_edges(edge.source, edge.target);

    bool found = false;
    int i = 0;
    while (!found && i < edges.value()->size()) {
      if (edge == edges.value()->at(i)) {
        found = true;

        chached_graph_.remove_edge(edge);

        bica_msgs::msg::GraphUpdate msg;
        msg.stamp = node_->now();
        msg.node_id = node_->get_name();
        msg.operation_type = bica_msgs::msg::GraphUpdate::REMOVE;
        msg.element_type = bica_msgs::msg::GraphUpdate::EDGE;
        msg.object = edge.to_string();

        if (!initialized_) {
          pending_updates_.push_back(msg);
        } else {
          update_pub_->publish(msg);
        }
      }
      i++;
    }
    return found;
  } else {
    return false;
  }
}

bool
GraphNode::exist_edge(const Edge & edge)
{
  return chached_graph_.exist_edge(edge);
}

std::optional<std::vector<Edge>*>
GraphNode::get_edges(const std::string & source, const std::string & target)
{
  return chached_graph_.get_edges(source, target);
}

std::string
GraphNode::to_string() const
{
  return chached_graph_.to_string();
}

void
GraphNode::from_string(const std::string & graph_str)
{
  chached_graph_.from_string(graph_str);
}

size_t
GraphNode::get_num_edges() const
{
  return chached_graph_.get_num_edges();
}

size_t
GraphNode::get_num_nodes() const
{
  return chached_graph_.get_num_nodes();
}

}  // namespace bica_graph
