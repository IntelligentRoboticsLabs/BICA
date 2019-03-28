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

#ifndef BICA_GRAPH_NODE_H
#define BICA_GRAPH_NODE_H

#include <list>
#include <memory>
#include <string>

#include <tf/tf.h>
#include <bica_msgs/Graph.h>

#include <bica_graph/macros.h>
#include <bica_graph/graph.h>


namespace bica_graph
{
class Relation;
class TFRelation;
class BicaGraph;

class Node : public std::enable_shared_from_this<Node>
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(Node)

  /// Default constructor.
  /**
   * Tipically, a Node is not created throught Graph::create_node
   * \param[in] id The id of the new node.
   * \param[in] type The type of the node.
   */
  Node(const std::string& id, const std::string& type, const ros::Time& time_stamp);

  /// get the id of the node.
  /**
  * \returns the id of the node.
  */
  std::string get_id() const {return id_;}

  /// get the type of the node.
  /**
  * \returns the type of the node.
  */
  std::string get_type() const {return type_;}

  /// get the time stamp of the node.
  /**
  * \returns the time stamp of the node.
  */
  ros::Time get_time_stamp() const {return ts_;}

  /// update de time stamp of the node.
  /**
  *
  */
  void update_time_stamp();

  /// Create a relation to other node.
  /**
  * This method creates the relation and links it to this node
  * \param[in] type The type of the relation.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<Relation> add_relation(const std::string& type,
    const std::shared_ptr<Node>& target);

  /// Create a TF relation to other node.
  /**
  * This method creates the TF relation and links it to this node
  * \param[in] tf The transform as a tf::Transform
  * \param[in] target The node related to this source node.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<TFRelation> add_tf_relation(
    const tf::Transform& tf,
    const std::shared_ptr<Node>& target);

  /// Create a relation to other node.
  /**
  * This method creates the relation and links it to this node
  * \param[in] type The type of the relation.
  * \param[in] target The node related to this source node.
  * \param[in] time_stamp The timestamp where the relation is created.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<Relation> add_relation(const std::string& type,
    const std::shared_ptr<Node>& target, const ros::Time& time_stamp);

  /// Create a TF relation to other node.
  /**
  * This method creates the TF relation and links it to this node
  * \param[in] tf The transform as a tf::Transform
  * \param[in] target The node related to this source node.
  * \param[in] time_stamp The timestamp where the relation is created.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<TFRelation> add_tf_relation(
    const tf::Transform& tf,
    const std::shared_ptr<Node>& target, const ros::Time& time_stamp);

  /// Remove a relation to other node.
  /**
   * \param[in] type The type of the relation.
   * \param[in] target The node related to this source node.
   * \param[in] target The timestamp where the relation is created.
  */
  void remove_relation(const std::string& type,
    const std::shared_ptr<Node>& target);

  /// Remove a ll the relation of a  node.
  /**
   *
  */
  void remove_all_relations();

  /// Remove a TF relation to other node.
  /**
   * \param[in] target The node related to this source node.
   * \param[in] target The timestamp where the relation is created.
  */
  void remove_tf_relation(const std::shared_ptr<Node>& target);

  /// annotate an incoming relation from other node.
  /**
    * \param[in] relation The relation to annotate
  */
  void add_incoming_relation(const std::shared_ptr<Relation>& relation);

  /// un-annotate an incoming relation from other node.
  /**
    * \param[in] relation The relation to un-annotate
  */
  void remove_incoming_relation(const std::shared_ptr<Relation>& relation);

  /// Get a relation pointer.
  /**
   * \param[in] target The target node of the relation.
   * \param[in] type The type of the  relation.
   * \returns The shared pointer to the relation. nullptr if it does not exist
  */
  std::shared_ptr<Relation> get_relation(
    const std::shared_ptr<Node>& target, const std::string& type);

  /// Get the list of relations of which this node is source
  /**
  * \returns the list of smart pointers to each relations
  */
  const std::list<std::shared_ptr<Relation>>& get_relations() const {return relations_out_;}

  /// Get the list of incoming relations to this node
  /**
  * \returns the list of smart pointers to each relations
  */
  const std::list<std::shared_ptr<Relation>>& get_incoming_relations() const {return relations_in_;}

  /// Create relations comming from a message
  /**
  * \param[in] node The msg that contains relations
  * \param[in] graph The graph where the nodo exists.
  */
  void add_relations_from_msg(const bica_msgs::Node& node, std::shared_ptr<BicaGraph> graph);

  /// Compare two nodes.
  /**
  * \param[in] other The node to compare.
  * \returns true if both nodes are equals
  */
  friend bool operator==(const Node& lhs, const Node& rhs);

  /// Compare tho nodes.
  /**
  * \param[in] other The node to compare.
  * \returns true if both nodes are not equals
  */
  friend bool operator!=(const Node& lhs, const Node& rhs);

  /// Fill a text stream for printing modes.
  /**
  * \param[in] lhs The stream to print to.
  * \param[in] other The node to stream.
  * \returns the ostream
  */
  friend std::ostream& operator<<(std::ostream& lhs, const Node& rhs);

protected:
  std::list<std::shared_ptr<Relation>> relations_out_;
  std::list<std::shared_ptr<Relation>> relations_in_;

  std::string id_;
  std::string type_;

  ros::Time ts_;
};

bool operator==(const Node& lhs, const Node& rhs);
bool operator!=(const Node& lhs, const Node& rhs);
std::ostream& operator<<(std::ostream& lhs, const Node& rhs);

}  // namespace bica_graph

#endif  // BICA_GRAPH_NODE_H
