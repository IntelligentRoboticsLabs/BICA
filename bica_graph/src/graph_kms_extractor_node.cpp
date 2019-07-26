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

#include <list>
#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"

#include "bica_planning/KMSClient.h"
#include "bica_graph/graph_client.h"

namespace bica_graph
{

std::vector<std::string> tokenize(const std::string& str, const std::string& delimiter = " ")
{
  std::vector<std::string> ret;
  size_t start = str.find_first_not_of(delimiter);
  size_t end = start;

  while (start != std::string::npos)
  {
    end = str.find(delimiter, start);
    ret.push_back(str.substr(start, end-start));
    start = str.find_first_not_of(delimiter, end);
  }

  return ret;
}

class GraphKMSExtractor: public bica_planning::KMSClient, public GraphClient
{
public:
  GraphKMSExtractor()
  : nh_("~")
  {
    nh_.getParam("interested_types", interested_types_);
    nh_.getParam("interested_predicates", interested_predicates_);
  }

  void update()
  {
    update_nodes();
    update_edges();
  }

private:
  void update_new_nodes()
  {
    for (std::string type : interested_types_)
    {
      for (std::string kms_instance : get_instances(type))
      {
        if (!graph_.exist_node(kms_instance))
        {
          graph_.add_node(kms_instance, type);
        }
      }
    }
  }

  bool is_node_interested(const std::string& type)
  {
    auto find_it = std::find(interested_types_.begin(), interested_types_.end(), type);
    return find_it != interested_types_.end();
  }

  bool node_in_kms(const std::string& type, const std::string& id)
  {
    bool found = false;
    std::vector<std::string> kms_instances = get_instances(type);

    int i = 0;
    while (i < kms_instances.size() && !found)
    {
      if (kms_instances[i++] == id)
        found = true;
    }

    return found;
  }

  void update_old_nodes()
  {
    std::list<std::string> nodes_to_remove;

    std::map<std::string, Node::SharedPtr>::const_iterator it;
    for (auto node : graph_.get_nodes())
    {
      std::string type = node.get_type();
      std::string id = node.get_id();

      if (!is_node_interested(type))
        continue;

      if (!node_in_kms(type, id))
        nodes_to_remove.push_back(id);
    }

    for (auto it = nodes_to_remove.begin(); it!= nodes_to_remove.end(); ++it)
      graph_.remove_node(*it);
  }

  void update_nodes()
  {
    update_new_nodes();
    update_old_nodes();
  }

  void update_new_edges()
  {
    for (std::string interested_predicates : interested_predicates_)
    {
      std::vector<std::string> predicates =
        search_predicates_regex(interested_predicates + " [[:print:]_]*");

      for (std::string predicate : predicates)
      {
        std::vector<std::string> tokens = tokenize(predicate);
        if ( (tokens.size() == 3) && exist_node(tokens[1]) && exist_node(tokens[2]) )
          add_edge(tokens[1], tokens[0], tokens[2]);
        if ( (tokens.size() == 2) && exist_node(tokens[1]) )
        {
          add_edge(tokens[1], tokens[0], tokens[1]);
        }
      }
    }
  }

  bool is_predicate_interested(const std::string& edge_data)
  {
    auto find_it = std::find(interested_predicates_.begin(), interested_predicates_.end(), edge_data);
    return find_it != interested_predicates_.end();
  }

  bool exists_as_predicate(const bica_graph::StringEdge& edge)
  {
    std::string predicate_first = edge.get();
    std::string predicate;

    if (edge.get_source() !=  edge.get_target())
      predicate = predicate_first + " " + edge.get_source() + " " + edge.get_target();
    else
      predicate = predicate_first + " " + edge.get_source();

    return !search_predicates_regex(predicate).empty();
  }

  void update_old_edges()
  {
    std::list<bica_graph::StringEdge> edges_to_remove;

    for (auto edge : graph_.get_string_edges())
    {
      if (is_predicate_interested(edge.get()) && !exists_as_predicate(edge))
        edges_to_remove.push_back(edge);
    }

    for (auto edge : edges_to_remove)
    {
      graph_.remove_edge(edge);
    }
  }


  void update_edges()
  {
    update_new_edges();
    update_old_edges();
  }

  ros::NodeHandle nh_;

  std::vector<std::string> interested_predicates_;
  std::vector<std::string> interested_types_;

  bica_graph::GraphClient graph_;
};

};  // namespace bica_graph

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "graph_kms_extractor");
  ros::NodeHandle n;

  bica_graph::GraphKMSExtractor graph_kms_extractor;

  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    graph_kms_extractor.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
