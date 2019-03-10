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

#ifndef BICA_GRAPH_TFRELATION_H
#define BICA_GRAPH_TFRELATION_H

#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <bica_msgs/TFRelation.h>

#include <bica_graph/macros.h>
#include <bica_graph/node.h>
#include <bica_graph/relation.h>

namespace bica_graph
{
class TFRelation : public std::enable_shared_from_this<TFRelation>, public Relation
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(TFRelation)

  /// Create a new relation.
  /**
  * Tipically, a Relation is not created throught Node::add_relation
  * \param[in] tf The transform of the relation
  * \param[in] source The source of the relation
  * \param[in] target The target of the relation
  */
  TFRelation(
    const geometry_msgs::TransformStamped& tf,
    const std::shared_ptr<Node>& source,
    const std::shared_ptr<Node>& target);

  /// get the transform of the relation.
  /**
  * \returns the transform stamped.
  */
  const geometry_msgs::TransformStamped& get_transform() const {return tf_;}

  /// Create a msg from this relation.
  /**
  * \returns a const pointer of the to a msg representing this relation
  */
  bica_msgs::TFRelationConstPtr transform_to_msg();

  /// Add this relation to a node message.
  /**
   * \param[in] node The node message to add this relation.
  */
  virtual void add_to_msg(bica_msgs::NodePtr node);


  /// Compare two relations.
  /**
  * \param[in] other The TF relations to compare.
  * \returns true if both TF relations are equals
  */
  friend bool operator==(const TFRelation& lhs, const TFRelation& rhs);

  /// Compare two relations.
  /**
  * \param[in] other The TF relations to compare.
  * \returns true if both TF relations are different
  */
  friend bool operator!=(const TFRelation& lhs, const TFRelation& rhs);

  /// Fill a text stream for printing tf relations.
  /**
  * \param[in] lhs The stream to print to.
  * \param[in] rhs The tf relation to stream.
  * \returns the ostream
  */
  friend std::ostream& operator<<(std::ostream& lhs, const TFRelation& rhs);

  /// Return the transform of this tf relation.
  /**
  * \returns a ref to the geometry_msgs::TransformStamped
  */
  geometry_msgs::TransformStamped& get_tf() {return tf_;}


protected:
  geometry_msgs::TransformStamped tf_;

};

bool operator==(const TFRelation& lhs, const TFRelation& rhs);
bool operator!=(const TFRelation& lhs, const TFRelation& rhs);
std::ostream& operator<<(std::ostream& lhs, const TFRelation& rhs);

}  // namespace bica_graph

#endif  // BICA_GRAPH_TFRELATION_H
