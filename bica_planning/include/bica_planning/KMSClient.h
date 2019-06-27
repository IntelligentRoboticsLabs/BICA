/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
#ifndef KMS_CLIENT_H
#define KMS_CLIENT_H

#include <ros/ros.h>

#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <utility>
#include <regex>
#include <map>
#include <string>
#include <vector>

namespace bica_planning
{
class KMSClient
{
public:
  KMSClient();

protected:
  bool add_instance(const std::string instance_type, const std::string attribute_name);
  bool add_goal(std::string goal);
  bool remove_current_goal();
  bool add_metric(std::string optimization, std::string metric);
  bool add_function(std::string metric_fact, float value);
  bool rm_function(std::string metric_fact, float value);
  bool rm_function_regex(std::regex re);
  bool update_function(std::string metric_fact, float value);
  bool update_function_kb(std::string function_str, float value, int update_type);

  bool remove_predicates_regex(std::regex re);
  std::vector<std::string> search_predicates_regex(std::regex re);
  bool add_predicate(std::string predicate);
  bool remove_predicate(const std::string& predicate);

  std::vector<std::string> get_instances(const std::string& instance_type);
  std::vector<std::string> tokenize(const std::string& text);
  std::vector<std::pair<std::string, float>> getCurrentFunctions();

  ros::NodeHandle nh_;
  ros::ServiceClient ku_client_, kq_client_, cg_client_, dp_client_, ki_client_,
    func_client_, kf_client_;

private:
  bool initPredicateArgs();
  bool initFunctionsArgs();

  const std::vector<diagnostic_msgs::KeyValue>& getPredicateArgs(const std::string predicate);

  std::string PredicateToString(const rosplan_knowledge_msgs::KnowledgeItem& knowledge);
  rosplan_knowledge_msgs::KnowledgeItem StringToPredicate(const std::string& predicate);
  rosplan_knowledge_msgs::KnowledgeItem StringToMetric(const std::string& metric);

  std::vector<std::string> getCurrentPredicates();

  std::map<std::string, std::vector<diagnostic_msgs::KeyValue> > domain_predicates_,
    func_predicates_;
};
};  // namespace bica_planning
#endif
