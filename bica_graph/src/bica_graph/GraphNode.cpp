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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <map>
#include <algorithm>
#include <sstream>
#include <vector>

#include "bica_graph/GraphNode.hpp"

#include "bica_msgs/msg/graph_update.hpp"


namespace bica_graph
{

GraphNode::GraphNode(const std::string & provided_node_name)
: seq_(0)
{
  std::cerr << "GraphNode::GraphNode start" << std::endl;
  node_ = std::make_shared<rclcpp::Node>(provided_node_name + "_graph");
  sync_node_ = std::make_shared<rclcpp::Node>(provided_node_name + "_graph_sync");

  initialized_ = node_->count_publishers("/graph_updates") == 0;
  last_ts_ = node_->now();

  using namespace std::placeholders;
  update_pub_ = node_->create_publisher<bica_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(1000).reliable());
  sync_update_pub_ = sync_node_->create_publisher<bica_msgs::msg::GraphUpdate>(
    "/graph_updates_sync", rclcpp::QoS(1000).reliable());

  update_sub_ = node_->create_subscription<bica_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(1000).reliable(),
    std::bind(&GraphNode::update_callback, this, _1));

  sync_update_sub_ = sync_node_->create_subscription<bica_msgs::msg::GraphUpdate>(
    "/graph_updates_sync", rclcpp::QoS(1000).reliable(),
    std::bind(&GraphNode::sync_update_callback, this, _1));

  if (!initialized_) {
    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = sync_node_->now();
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::REQSYNC;
    msg.element_type = bica_msgs::msg::GraphUpdate::GRAPH;
    msg.object = graph_.to_string();
    msg.seq = 0;
    sync_update_pub_->publish(msg);
  }

  sync_spin_t_ = std::thread([this] {
        rclcpp::spin(this->sync_node_);
      });
  sync_spin_t_.detach();
}

void
GraphNode::sync_update_callback(const bica_msgs::msg::GraphUpdate::SharedPtr msg)
{
  auto update = *msg;

  if (update.operation_type == bica_msgs::msg::GraphUpdate::REQSYNC) {
    if (initialized_) {
      // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " <<
      //   update.node_id << "]\t" << seq_ << "\tREQSYNC" <<std::endl;
      bica_msgs::msg::GraphUpdate msg;
      msg.stamp = last_ts_ + rclcpp::Duration(0.0, 1.0);  // Dt
      msg.node_id = node_->get_name();
      msg.operation_type = bica_msgs::msg::GraphUpdate::SYNC;
      msg.element_type = bica_msgs::msg::GraphUpdate::GRAPH;
      msg.object = graph_.to_string();
      msg.seq = seq_;
      sync_update_pub_->publish(msg);
    }
  } else if (update.element_type == bica_msgs::msg::GraphUpdate::GRAPH) {
    if (!initialized_) {
      graph_.from_string(update.object);
      seq_ = update.seq;
      initialized_ = true;
      last_ts_ = node_->now();
    }
  }
}

void
GraphNode::update_callback(const bica_msgs::msg::GraphUpdate::SharedPtr msg)
{
  auto update = *msg;
  last_ts_ = rclcpp::Time(update.stamp);

  if (update.element_type == bica_msgs::msg::GraphUpdate::NODE) {
    Node node;
    node.from_string(update.object);
    seq_ = update.seq;
    if (update.operation_type == bica_msgs::msg::GraphUpdate::ADD) {
      graph_.add_node(node);
      // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() <<
      //   ", " << update.node_id << "]\t" << seq_ << "\tADD " <<node.to_string() <<std::endl;
    } else if (update.operation_type == bica_msgs::msg::GraphUpdate::REMOVE) {
      graph_.remove_node(node.name);
      // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " <<
      //   update.node_id << "]\t" << seq_ << "\tREMOVE " <<node.to_string() <<std::endl;
    }
    last_ts_ = node_->now();
  } else if (update.element_type == bica_msgs::msg::GraphUpdate::EDGE) {
    Edge edge;
    edge.from_string(update.object);
    seq_ = update.seq;

    if (update.operation_type == bica_msgs::msg::GraphUpdate::ADD) {
      graph_.add_edge(edge);
      // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " <<
      //   update.node_id << "]\t" << seq_ << "\tADD " << edge.to_string() << std::endl;
    } else if (update.operation_type == bica_msgs::msg::GraphUpdate::REMOVE) {
      graph_.remove_edge(edge);
      // std::cout << "[" << std::fixed << rclcpp::Time(update.stamp).seconds() << ", " <<
      //   update.node_id << "]\t" << seq_ << "\tREMOVE " <<edge.to_string() <<std::endl;
    }
    last_ts_ = node_->now();
  }
}

bool
GraphNode::add_node(const Node & node)
{
  rclcpp::spin_some(node_);

  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "Operation before initialization");
  }

  if (graph_.add_node(node)) {
    seq_++;

    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = node_->now();
    msg.seq = seq_;
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::ADD;
    msg.element_type = bica_msgs::msg::GraphUpdate::NODE;
    msg.object = node.to_string();

    update_pub_->publish(msg);
    rclcpp::spin_some(node_);
    return true;
  } else {
    return false;
  }
}

bool
GraphNode::remove_node(const std::string node)
{
  rclcpp::spin_some(node_);

  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "Operation before initialization");
  }

  if (graph_.remove_node(node)) {
    seq_++;

    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = node_->now();
    msg.seq = seq_;
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::REMOVE;
    msg.element_type = bica_msgs::msg::GraphUpdate::NODE;
    msg.object = Node{node, "no_type"}.to_string();

    update_pub_->publish(msg);
    rclcpp::spin_some(node_);
    return true;
  } else {
    return false;
  }
}

bool
GraphNode::exist_node(const std::string node)
{
  rclcpp::spin_some(node_);
  return graph_.exist_node(node);
}

boost::optional<Node>
GraphNode::get_node(const std::string node)
{
  rclcpp::spin_some(node_);
  return graph_.get_node(node);
}

bool
GraphNode::add_edge(const Edge & edge)
{
  rclcpp::spin_some(node_);
  if (graph_.add_edge(edge)) {
    seq_++;

    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = node_->now();
    msg.seq = seq_;
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::ADD;
    msg.element_type = bica_msgs::msg::GraphUpdate::EDGE;
    msg.object = edge.to_string();

    update_pub_->publish(msg);
    rclcpp::spin_some(node_);
    return true;
  } else {
    return false;
  }
}

bool
GraphNode::remove_edge(const Edge & edge)
{
  rclcpp::spin_some(node_);
  if (graph_.remove_edge(edge)) {
    seq_++;

    bica_msgs::msg::GraphUpdate msg;
    msg.stamp = node_->now();
    msg.seq = seq_;
    msg.node_id = node_->get_name();
    msg.operation_type = bica_msgs::msg::GraphUpdate::REMOVE;
    msg.element_type = bica_msgs::msg::GraphUpdate::EDGE;
    msg.object = edge.to_string();

    update_pub_->publish(msg);
    rclcpp::spin_some(node_);
    return true;
  } else {
    return false;
  }
}

bool
GraphNode::exist_edge(const Edge & edge)
{
  rclcpp::spin_some(node_);
  return graph_.exist_edge(edge);
}

boost::optional<std::vector<Edge> *>
GraphNode::get_edges(const std::string & source, const std::string & target)
{
  rclcpp::spin_some(node_);
  return graph_.get_edges(source, target);
}

std::string
GraphNode::to_string() const
{
  rclcpp::spin_some(node_);
  return graph_.to_string();
}

void
GraphNode::from_string(const std::string & graph_str)
{
  graph_.from_string(graph_str);
}

size_t
GraphNode::get_num_edges() const
{
  rclcpp::spin_some(node_);
  return graph_.get_num_edges();
}

size_t
GraphNode::get_num_nodes() const
{
  rclcpp::spin_some(node_);
  return graph_.get_num_nodes();
}

const std::map<std::string, Node> &
GraphNode::get_nodes()
{
  rclcpp::spin_some(node_);
  return graph_.get_nodes();
}

const std::map<ConnectionT, std::vector<Edge>> &
GraphNode::get_edges()
{
  rclcpp::spin_some(node_);
  return graph_.get_edges();
}

std::vector<std::string>
GraphNode::get_node_names_by_id(const std::string & expr)
{
  return graph_.get_node_names_by_id(expr);
}

std::vector<std::string>
GraphNode::get_node_names_by_type(const std::string & type)
{
  return graph_.get_node_names_by_type(type);
}

std::vector<Edge>
GraphNode::get_edges_from_node(const std::string & node_src_id, const std::string & type)
{
  return graph_.get_edges_from_node(node_src_id, type);
}

std::vector<Edge>
GraphNode::get_edges_from_node_by_data(
  const std::string & node_src_id,
  const std::string & expr,
  const std::string & type)
{
  return graph_.get_edges_from_node_by_data(node_src_id, expr, type);
}

std::vector<Edge>
GraphNode::get_edges_by_data(const std::string & expr, const std::string & type)
{
  return graph_.get_edges_by_data(expr, type);
}

}  // namespace bica_graph
