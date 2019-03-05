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

#ifndef BICA_GRAPH_RELATION_H
#define BICA_GRAPH_RELATION_H

#include <memory>
#include <string>

#include <bica_graph/macros.h>

namespace bica_graph
{

class Node;

class Relation : public std::enable_shared_from_this<Relation>
{
public:
  BICA_GRAPH_SMART_PTR_DEFINITIONS(Relation)

  /// Create a new relation.
  /**
  */
  Relation(const std::string& type, const std::shared_ptr<Node>& source, const std::shared_ptr<Node>& target);

  /// Create a destructor .
  /**
   * This is necessary to do Relation class polymorfic
  */
  virtual ~Relation() = default;

  /// get the type of the relation.
  /**
  * \returns the type of the relation.
  */
  std::string get_type() const {return type_;}

  /// <get the source of the relation.
  /**
  * \returns the smart pointer to the source node.
  */
  const std::shared_ptr<Node>& get_source() const {return source_;}

  /// <get the target of the relation.
  /**
  * \returns the smart pointer to the target node.
  */
  const std::shared_ptr<Node>& get_target() const {return target_;}

protected:
  std::shared_ptr<Node> target_;
  std::shared_ptr<Node> source_;
  std::string type_;
};

}  // namespace bica_graph

#endif  // BICA_GRAPH_RELATION_H
