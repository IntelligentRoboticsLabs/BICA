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

#include "bica_graph/graph_server.h"

namespace bica_graph
{

GraphServer::GraphServer()
{
  graph_ = std::make_shared<Graph>();

  graph_sub_ = nh_.advertise<bica_msgs::Graph>("/graph", 1, true);
  update_srv_server_ = nh_.advertiseService(
    "/update_graph", &GraphServer::update_service_handler, this);
}

void
GraphServer::publish_graph()
{
  bica_msgs::Graph msg;
  graph_to_msg(*graph_, &msg);
  graph_sub_.publish(msg);
}

bool
GraphServer::handle_node_update(const bica_msgs::GraphUpdate& update)
{
  bool ret = true;

  switch (update.update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      if (!graph_->exist_node(update.node_id))
      {
        graph_->add_node(update.node_id, update.node_type);
        publish_graph();
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      if (graph_->exist_node(update.node_id))
      {
        graph_->remove_node(update.node_id);
        publish_graph();
      }
  }

  return ret;
}

bool
GraphServer::handle_string_edge_update(const bica_msgs::GraphUpdate& update)
{
  bool ret = true;

  switch (update.update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      if (!graph_->exist_edge<std::string>(update.edge_source, update.edge_target, update.edge_type))
      {
        graph_->add_edge(update.edge_source, update.edge_type, update.edge_target);
        publish_graph();
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      if (graph_->exist_edge<std::string>(update.edge_source, update.edge_target, update.edge_type))
      {
        graph_->remove_edge(update.edge_source, update.edge_target, update.edge_type);
        publish_graph();
      }
  }

  return ret;
}

bool
GraphServer::handle_double_edge_update(const bica_msgs::GraphUpdate& update)
{
  bool ret = true;

  switch (update.update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      if (!graph_->exist_edge<double>(update.edge_source, update.edge_target, update.edge_double))
      {
        graph_->add_edge(update.edge_source, update.edge_double, update.edge_target);
        publish_graph();
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      if (graph_->exist_edge<double>(update.edge_source, update.edge_target, update.edge_double))
      {
        graph_->remove_edge(update.edge_source, update.edge_target, update.edge_double);
        publish_graph();
      }
  }

  return ret;
}

bool
GraphServer::handle_tf_edge_update(const bica_msgs::GraphUpdate& update)
{
  bool ret = true;

  switch (update.update_type)
  {
    case bica_msgs::GraphUpdate::ADD:
      if (!graph_->exist_edge<tf::Transform>(update.edge_source, update.edge_target))
      {
        graph_->add_tf_edge(update.edge_source, update.edge_target);
        publish_graph();
      }
      break;
    case bica_msgs::GraphUpdate::REMOVE:
      if (graph_->exist_edge<tf::Transform>(update.edge_source, update.edge_target))
      {
        graph_->remove_edge<tf::Transform>(update.edge_source, update.edge_target);
        publish_graph();
      }
  }

  return ret;
}

bool
GraphServer::update_service_handler(bica_msgs::UpdateGraph::Request  &req,
                            bica_msgs::UpdateGraph::Response &rep)
{
  bool success = true;

  switch (req.update.element_type)
  {
    case bica_msgs::GraphUpdate::NODE:
      success = handle_node_update(req.update);
      break;
    case bica_msgs::GraphUpdate::STRING_EDGE:
      success = handle_string_edge_update(req.update);
      break;
    case bica_msgs::GraphUpdate::DOUBLE_EDGE:
      success = handle_double_edge_update(req.update);
      break;
    case bica_msgs::GraphUpdate::TF_EDGE:
      success = handle_tf_edge_update(req.update);
      break;
  }

  rep.success = success;

  return true;
}

void
GraphServer::graph_callback(const bica_msgs::Graph::ConstPtr& msg)
{
  msg_to_graph(*msg, graph_);
}

}  // namespace bica_graph
