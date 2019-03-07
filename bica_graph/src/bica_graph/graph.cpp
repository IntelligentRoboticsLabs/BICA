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

#include <bica_graph/graph.h>

using bica_graph::BicaGraph;

BicaGraph::BicaGraph()
: locked_(false)
{
}

BicaGraph::BicaGraph(const std::shared_ptr<Node>& node)
: locked_(true)
{
  nodes_.push_back(node);
}

size_t
BicaGraph::count_nodes() const
{
  return nodes_.size();
}

bool
BicaGraph::is_sub_graph() const
{
  return locked_;
}

std::shared_ptr<bica_graph::Node>
BicaGraph::create_node(const std::string& id, const std::string& type)
{
  auto node = std::make_shared<bica_graph::Node>(id, type);
  nodes_.push_back(node);

  return node;
}

BicaGraph&
BicaGraph::operator=(const BicaGraph& other)
{
  // (fmrico): Copy a graph is copying the list of references. Both graphs
  //           are linked. This is very dangerous and only must be done when
  //           copying for temporary grpahs
  nodes_ = other.get_nodes();

  return *this;
}


std::shared_ptr<bica_graph::Node>
BicaGraph::get_node(const std::string& id)
{
  std::shared_ptr<bica_graph::Node> ret = nullptr;

  for (auto it = nodes_.begin(); it!= nodes_.end(); ++it)
  {
    if ((*it)->get_id() == id)
    {
      ret = *it;
    }
  }

  return ret;
}

bool bica_graph::operator==(const BicaGraph& lhs, const BicaGraph& rhs)
{
  if (lhs.nodes_.size() != rhs.nodes_.size())
  {
    return false;
  }

  for (std::pair<std::list<std::shared_ptr<Node>>::const_iterator, std::list<std::shared_ptr<Node>>::const_iterator>
        it(lhs.nodes_.begin(), rhs.nodes_.begin());
       it.first != lhs.nodes_.end() && it.second != rhs.nodes_.end();
       ++it.first, ++it.second)
  {
    if (**it.first != **it.second)
    {
      return false;
    }
  }

  return true;
}

std::ostream& bica_graph::operator<<(std::ostream& lhs, const BicaGraph& rhs)
{
  lhs << "======================================================" << std::endl;
  lhs << "Number of nodes: " << rhs.nodes_.size() << std::endl;
  for (auto it = rhs.nodes_.begin(); it!= rhs.nodes_.end(); ++it)
  {
    lhs << **it << std::endl;
  }
  lhs << "======================================================" << std::endl;
  return lhs;
}
