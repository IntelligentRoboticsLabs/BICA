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

#include <geometry_msgs/TransformStamped.h>

#include <bica_graph/macros.h>
#include <bica_graph/graph.h>


namespace bica_graph
{
class Relation;
class TFRelation;

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
  Node(const std::string& id, const std::string& type);

  /// <get the id of the node.
  /**
  * \returns the id of the node.
  */
  std::string get_id() const {return id_;}

  /// <get the type of the node.
  /**
  * \returns the type of the node.
  */
  std::string get_type() const {return type_;}

  /// Create a relation to other node.
  /**
  * This method creates the relation and links it to this node
  * \param[in] type The type of the relation.
  * \param[in] target The node related to this source node.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<Relation> add_relation(const std::string& type, const std::shared_ptr<Node>& target);

  /// Create a TF relation to other node.
  /**
  * This method creates the TF relation and links it to this node
  * \param[in] tf The transform as a geometry_msgs::TransformStamped
  * \param[in] target The node related to this source node.
  * \returns the pointer of the new created relation
  */
  std::shared_ptr<TFRelation> add_tf_relation(
    const geometry_msgs::TransformStamped& tf,
    const std::shared_ptr<Node>& target);

  /// Get the list of relations of which this node is source
  /**
  * \returns the list of smart pointers to each relations
  */
  const std::list<std::shared_ptr<Relation>>& get_relations() const {return relations_;}


protected:
  std::list<std::shared_ptr<Relation>> relations_;

  std::string id_;
  std::string type_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_NODE_H
