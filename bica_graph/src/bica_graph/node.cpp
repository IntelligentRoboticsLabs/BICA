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
#include <utility>
#include <list>

#include <bica_graph/node.h>

#include <bica_graph/relation.h>
#include <bica_graph/tf_relation.h>

using bica_graph::Node;

Node::Node(const std::string& id, const std::string& type, const ros::Time& time_stamp)
: id_(id), type_(type), ts_(time_stamp)
{
}

void
Node::update_time_stamp()
{
  ts_ = ros::Time::now();
}

std::shared_ptr<bica_graph::Relation>
Node::add_relation(const std::string& type, const std::shared_ptr<Node>& target)
{
  return add_relation(type, target, ros::Time::now());
}

std::shared_ptr<bica_graph::TFRelation>
Node::add_tf_relation(
  const tf::Transform& tf,
  const std::shared_ptr<Node>& target)
{
  return add_tf_relation(tf, target, ros::Time::now());
}

std::shared_ptr<bica_graph::Relation>
Node::add_relation(const std::string& type, const std::shared_ptr<Node>& target,
  const ros::Time& time_stamp)
{
  auto relation = get_relation(target, type);

  if (relation == nullptr)
  {
    relation = std::make_shared<bica_graph::Relation>(type, shared_from_this(), target, time_stamp);
    relations_out_.push_back(relation);
    target->add_incoming_relation(relation);
  }
  else
  {
    relation->update_time_stamp();
  }

  return relation;
}

void
Node::remove_all_relations()
{
  for (auto it = relations_out_.begin(); it!= relations_out_.end(); ++it)
  {
    (*it)->get_target()->remove_incoming_relation(*it);
  }
  relations_out_.clear();

  while (!relations_in_.empty())
  {
    auto it = relations_in_.front();
    auto source = it->get_source();
    source->remove_relation(it->get_type(), it->get_target());
  }

  relations_in_.clear();
}

std::shared_ptr<bica_graph::TFRelation>
Node::add_tf_relation(
  const tf::Transform& tf,
  const std::shared_ptr<Node>& target, const ros::Time& time_stamp)
{
  auto relation = std::dynamic_pointer_cast<bica_graph::TFRelation>(get_relation(target, "tf"));

  if (relation == nullptr)
  {
    relation = std::make_shared<bica_graph::TFRelation>(tf, shared_from_this(),
      target, ros::Time::now());
    relations_out_.push_back(relation);
    target->add_incoming_relation(relation);
  }
  else
  {
    relation->update_time_stamp();
  }

  return relation;
}

void
Node::remove_relation(const std::string& type,
  const std::shared_ptr<Node>& target)
{
    auto relation = get_relation(target, type);
    target->remove_incoming_relation(relation);

    auto it = relations_out_.begin();
    while (it!= relations_out_.end())
    {
      if ((*it)->get_target() == target && (*it)->get_type() == type)
      {
        it = relations_out_.erase(it);
      }
      else
        ++it;
    }
}

void
Node::remove_tf_relation(const std::shared_ptr<Node>& target)
{
  remove_relation("tf", target);
}

std::shared_ptr<bica_graph::Relation>
Node::get_relation(const std::shared_ptr<Node>& target, const std::string& type)
{
  std::shared_ptr<bica_graph::Relation> ret = nullptr;

  for (auto it = relations_out_.begin(); it!= relations_out_.end(); ++it)
  {
    if ((*it)->get_target() == target && (*it)->get_type() == type)
    {
      ret = *it;
    }
  }

  return ret;
}

void
Node::add_relations_from_msg(const bica_msgs::Node& node, std::shared_ptr<bica_graph::BicaGraph> graph)
{
  for (int i = 0; i < node.relations.size(); i++)
  {
    assert(node.relations[i].source == id_);
    add_relation(node.relations[i].type, graph->get_node(node.relations[i].target),
      node.relations[i].stamp);
  }

  for (int i = 0; i < node.tf_relations.size(); i++)
  {
    assert(node.tf_relations[i].source == id_);
    tf::Transform tf;
    tf::transformMsgToTF(node.tf_relations[i].transform, tf);
    add_tf_relation(tf, graph->get_node(node.relations[i].target),
      node.tf_relations[i].stamp);
  }
}

void
Node::add_incoming_relation(const std::shared_ptr<bica_graph::Relation>& relation)
{
  relations_in_.push_back(relation);
}

void
Node::remove_incoming_relation(const std::shared_ptr<bica_graph::Relation>& relation)
{
  auto it = relations_in_.begin();
  while (it != relations_in_.end())
  {
    if (*(*it) == *relation)
      it = relations_in_.erase(it);
    else
      ++it;
  }
}

bool bica_graph::operator==(const Node& lhs, const Node& rhs)
{
  if (lhs.id_ != rhs.id_)
  {
    return false;
  }

  if (lhs.type_ != rhs.type_)
  {
    return false;
  }

  if (lhs.relations_out_.size() != rhs.relations_out_.size())
  {
    return false;
  }

  for (std::pair<
        std::list<std::shared_ptr<Relation>>::const_iterator, std::list<std::shared_ptr<Relation>>::const_iterator>
        it(lhs.relations_out_.begin(), rhs.relations_out_.begin());
       it.first != lhs.relations_out_.end() && it.second != rhs.relations_out_.end();
       ++it.first, ++it.second)
  {
    if (**it.first != **it.second)
    {
      return false;
    }
  }

  return true;
}


bool bica_graph::operator!=(const Node& lhs, const Node& rhs)
{
  return !(lhs == rhs);
}

std::ostream& bica_graph::operator<<(std::ostream& lhs, const Node& rhs)
{
  lhs << "Node [" << rhs.id_ << "(" << rhs.type_ <<")] "<< std::endl;
  lhs << "Number of relation: " << rhs.relations_out_.size() << std::endl;
  for (auto it = rhs.relations_out_.begin(); it!= rhs.relations_out_.end(); ++it)
  {
    if ((*it)->get_type() == "tf")
    {
      auto r =  std::dynamic_pointer_cast<bica_graph::TFRelation>(*it);
      lhs << *r << std::endl;
    }
    else
    {
      lhs << **it << std::endl;
    }
  }
  return lhs;
}
