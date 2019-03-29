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

#ifndef BICA_GRAPH_GRAPH_HANDLER_H
#define BICA_GRAPH_GRAPH_HANDLER_H

#include <ros/ros.h>

#include <string>

#include <bica_graph/graph.h>
#include <bica_graph/relation.h>
#include <bica_graph/tf_relation.h>
#include <bica_msgs/Graph.h>
#include <bica_msgs/GraphUpdate.h>
#include <bica_graph/graph_listener.h>

namespace bica_graph
{

class GraphHandler
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(GraphHandler)

  /// Create a graph hanlder.
  /**
  * \param[in] nh The NodeHandle
  * \param[in] handler_id An optional parameter used to differenciate graph handlers
      in the same node.
  */
  explicit GraphHandler(ros::NodeHandle& nh, std::string handler_id = "default_id");

  /// Create a node and insert it in the graph.
  /**
  * \param[in] id The id of the new node.
  * \param[in] type The type of the node.
  */
  std::shared_ptr<Node> create_node(const std::string& id,
    const std::string& type);

  /// Remove a node.
  /**
  * \param[in] id The id of the node to remove.
  */
  void remove_node(const std::string& id);

  /// Get the number of nodes in a graph.
  /**
   * \returns The number of nodes
  */
  size_t count_nodes() const;

  /// Get a node pointer.
  /**
   * \param[in] id The id of the  node.
   * \returns The shared pointer to the node. nullptr if it does not exist
  */
  std::shared_ptr<bica_graph::Node> get_node(const std::string& id) const;

  /// Create a relation to other node.
  /**
  * This method creates the relation and links it to this node
  * \param[in] source The source of the relation.
  * \param[in] type The type of the relation.
  * \param[in] target The target of the relation.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<Relation> add_relation(const std::string& source,
    const std::string& type,
    const std::string& target);

  /// Create a TF relation to other node.
  /**
  * This method creates the relation and links it to this node
  * \param[in] source The source of the relation.
  * \param[in] tf The transform
  * \param[in] target The target of the relation.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<Relation> add_tf_relation(const std::string& source,
    const tf::Transform& tf,
    const std::string& target);

  /// get a TF relation .
  /**
  * \param[in] source The source of the relation.
  * \param[in] target The target of the relation.
  * \returns the pointer of the tf  relation
  */
  std::shared_ptr<TFRelation> get_tf_relation(const std::string& source,
    const std::string& target);

  /// Remove a relation.
  /**
  * \param[in] id The id of the relation to remove.
  */
  void remove_relation(const std::string& source,
    const std::string& type,
    const std::string& target);

  /// Remove a tf relation.
  /**
  * \param[in] id The id of the relation to remove.
  */
  void remove_tf_relation(const std::string& source,
    const std::string& target);

  /// Checks if the graph contains a node by id.
  /**
  * \param[in] node_id The id of the node to loof for.
  * \returns true if the node is in the graph
  */
  bool contains_node(const std::string& node_id);

  /// Checks if the graph contains a tf relation.
  /**
  * \param[in] source The source of the relation.
  * \param[in] type The type of the relation.
  * \param[in] target The target of the relation.
  * \returns true if the relation is in the graph
  */
  bool contains_relation(const std::string& source,
    const std::string& type,
    const std::string& target);

  /// Checks if the graph contains a tf relation.
  /**
  * \param[in] source The source of the relation.
  * \param[in] target The target of the relation.
  * \returns true if the relation is in the graph
  */
  bool contains_tf_relation(const std::string& source,
    const std::string& target);
private:
  void graph_update_callback(const ros::MessageEvent<bica_msgs::GraphUpdate const>& event);

  void update_node(const bica_msgs::GraphUpdateConstPtr& msg);
  void update_relation(const bica_msgs::GraphUpdateConstPtr& msg);
  void update_tf_relation(const bica_msgs::GraphUpdateConstPtr& msg);


  ros::NodeHandle nh_;
  std::string handler_id_;
  BicaGraph::SharedPtr graph_;

  ros::Subscriber graph_update_sub_;
  ros::Publisher graph_update_pub_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_GRAPH_HANDLER_H
