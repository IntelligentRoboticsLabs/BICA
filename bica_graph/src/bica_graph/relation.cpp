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

#include <bica_graph/relation.h>
#include <bica_graph/node.h>

using bica_graph::Relation;

Relation::Relation(const std::string& type, const std::shared_ptr<Node>& source, const std::shared_ptr<Node>& target)
: source_(source), target_(target), type_(type)
{
}

bica_msgs::RelationConstPtr
Relation::transform_to_msg()
{
  bica_msgs::RelationPtr msg (new bica_msgs::Relation());

  msg->type = type_;
  msg->source = source_->get_id();
  msg->target = target_->get_id();

  return msg;
}

void
Relation::add_to_msg(bica_msgs::NodePtr node)
{
  node->relations.push_back(*this->transform_to_msg());
}


bool bica_graph::operator==(const Relation& lhs, const Relation& rhs)
{
  if ((*lhs.target_).get_id() != (*lhs.target_).get_id())
  {
    return false;
  }

  if ((*lhs.source_).get_id() != (*rhs.source_).get_id())
  {
    return false;
  }

  if (lhs.type_ != rhs.type_)
  {
    return false;
  }

  return true;
}

bool bica_graph::operator!=(const Relation& lhs, const Relation& rhs)
{
  return !(lhs == rhs);
}

std::ostream& bica_graph::operator<<(std::ostream& lhs, const Relation& rhs)
{
  lhs << "Relation [" << rhs.type_ <<"] " <<
    (*rhs.source_).get_id() << " -> " << (*rhs.target_).get_id() << std::endl;
  return lhs;
}
