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
#include "bica_planning/KMSClient.h"

#include <string>
#include <vector>

namespace bica_planning
{
KMSClient::KMSClient()
  : nh_()
  , ku_client_(nh_.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update"))
  , kq_client_(nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions"))
  , cg_client_(nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals"))
  , dp_client_(
      nh_.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/rosplan_knowledge_base/domain/predicates"))
  , func_client_(
      nh_.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/rosplan_knowledge_base/domain/functions"))
  , kf_client_(
      nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/functions"))
  , ki_client_(nh_.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/rosplan_knowledge_base/state/instances"))
{
  while (!ros::service::waitForService("/rosplan_knowledge_base/update", 1000))
    ROS_INFO("Waiting for /rosplan_knowledge_base/update service..");
  while (!ros::service::waitForService("/rosplan_knowledge_base/state/propositions", 1000))
    ROS_INFO("Waiting for /rosplan_knowledge_base/state/propositions service..");
  while (!ros::service::waitForService("/rosplan_knowledge_base/state/goals", 1000))
    ROS_INFO("Waiting for /rosplan_knowledge_base/state/goals service..");
  while (!ros::service::waitForService("/rosplan_knowledge_base/domain/predicates", 1000))
    ROS_INFO("Waiting for /rosplan_knowledge_base/domain/predicates service..");
  while (!ros::service::waitForService("/rosplan_knowledge_base/state/instances", 1000))
    ROS_INFO("Waiting for /rosplan_knowledge_base/state/instances service..");

  if (!initPredicateArgs())
    ROS_ERROR("Error initializing predicate args");
  if (!initFunctionsArgs())
    ROS_ERROR("Error initializing function args");
}

std::vector<std::string> KMSClient::tokenize(const std::string& text)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos)
  {
    end = text.find(" ", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}

bool KMSClient::initPredicateArgs()
{
  rosplan_knowledge_msgs::GetDomainAttributeService srv;

  if (!dp_client_.call(srv))
  {
    ROS_ERROR("Error calling service for initializing function args");
    return false;
  }
  else
  {
    for (int i = 0; i < srv.response.items.size(); i++)
      domain_predicates_[srv.response.items[i].name] = srv.response.items[i].typed_parameters;
    return true;
  }
}

bool KMSClient::initFunctionsArgs()
{
  rosplan_knowledge_msgs::GetDomainAttributeService srv;

  if (!func_client_.call(srv))
  {
    ROS_ERROR("Error calling service for initializing predicate args");
    return false;
  }
  else
  {
    for (int i = 0; i < srv.response.items.size(); i++)
      func_predicates_[srv.response.items[i].name] = srv.response.items[i].typed_parameters;
    return true;
  }
}

const std::vector<diagnostic_msgs::KeyValue>& KMSClient::getPredicateArgs(const std::string predicate)
{
  return domain_predicates_[predicate];
}

std::string KMSClient::PredicateToString(const rosplan_knowledge_msgs::KnowledgeItem& knowledge)
{
  std::string ret;

  ret = knowledge.attribute_name;
  for (int i = 0; i < knowledge.values.size(); i++)
    ret = ret + " " + knowledge.values[i].value;

  return ret;
}

rosplan_knowledge_msgs::KnowledgeItem KMSClient::StringToPredicate(const std::string& predicate)
{
  std::vector<std::string> tokens = tokenize(predicate);

  if (domain_predicates_[tokens[0]].size() != tokens.size() - 1)
  {
    ROS_ERROR("KMSClient::StringToPredicate Predicate [%s] num args(%zu) is not equal to one stored (%zu)",
              predicate.c_str(), tokens.size() - 1, domain_predicates_[tokens[0]].size());
    return rosplan_knowledge_msgs::KnowledgeItem();  // void
  }

  rosplan_knowledge_msgs::KnowledgeItem ret;
  ret.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
  ret.attribute_name = tokens[0];
  ret.function_value = 0.0;
  ret.is_negative = false;

  for (int i = 0; i < domain_predicates_[tokens[0]].size(); i++)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = domain_predicates_[tokens[0]][i].key;
    kv.value = tokens[i + 1];
    ret.values.push_back(kv);
  }

  return ret;
}

rosplan_knowledge_msgs::KnowledgeItem KMSClient::StringToMetric(const std::string& metric)
{
  std::vector<std::string> tokens = tokenize(metric);

  if (func_predicates_[tokens[0]].size() != tokens.size() - 1)
  {
    ROS_ERROR("KMSClient::StringToPredicate Metric [%s] num args(%zu) is not equal to one stored (%zu)",
              metric.c_str(), tokens.size() - 1, func_predicates_[tokens[0]].size());
    return rosplan_knowledge_msgs::KnowledgeItem();  // void
  }

  rosplan_knowledge_msgs::KnowledgeItem ret;
  ret.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
  ret.attribute_name = tokens[0];
  ret.function_value = 0.0;
  ret.is_negative = false;

  for (int i = 0; i < func_predicates_[tokens[0]].size(); i++)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = func_predicates_[tokens[0]][i].key;
    kv.value = tokens[i + 1];
    ret.values.push_back(kv);
  }
  return ret;
}

bool KMSClient::add_instance(const std::string instance_type, const std::string attribute_name)
{
  rosplan_knowledge_msgs::KnowledgeItem add_instance;
  add_instance.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
  add_instance.instance_type = instance_type;
  add_instance.instance_name = attribute_name;
  rosplan_knowledge_msgs::KnowledgeUpdateService add_instance_instance;
  add_instance_instance.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
  add_instance_instance.request.knowledge = add_instance;

  if (!ku_client_.call(add_instance_instance))
  {
    ROS_ERROR("Could not add the instance [%s %s]", instance_type.c_str(), attribute_name.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Instance [%s %s] added", instance_type.c_str(), attribute_name.c_str());
    return true;
  }
}

bool KMSClient::add_goal(std::string goal)
{
  rosplan_knowledge_msgs::KnowledgeItem add_fact_v = StringToPredicate(goal);
  if (add_fact_v.attribute_name == "")
    return false;

  rosplan_knowledge_msgs::KnowledgeUpdateService add_goal_instance;
  add_goal_instance.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
  add_goal_instance.request.knowledge = add_fact_v;

  if (!ku_client_.call(add_goal_instance))
  {
    ROS_ERROR("Could not add the goal [%s]", goal.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Goal [%s] added", goal.c_str());
    return true;
  }
}

bool KMSClient::add_metric(std::string optimization, std::string metric)
{
  rosplan_knowledge_msgs::ExprComposite expr_array;
  rosplan_knowledge_msgs::ExprBase expr_base;
  rosplan_knowledge_msgs::KnowledgeItem add_fact_v;

  add_fact_v.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::EXPRESSION;
  add_fact_v.optimization = optimization;
  expr_base.expr_type = rosplan_knowledge_msgs::ExprBase::FUNCTION;
  expr_base.function.name = metric;
  expr_array.tokens.push_back(expr_base);
  add_fact_v.expr = expr_array;

  rosplan_knowledge_msgs::KnowledgeUpdateService add_metric_instance;
  add_metric_instance.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_METRIC;
  add_metric_instance.request.knowledge = add_fact_v;

  if (!ku_client_.call(add_metric_instance))
  {
    ROS_ERROR("Could not add the metric [%s]", metric.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Metric [%s] added", metric.c_str());
    return true;
  }
}

bool KMSClient::update_function_kb(std::string function_str, float value, int update_type)
{
  rosplan_knowledge_msgs::KnowledgeItem funct_item = StringToMetric(function_str);
  funct_item.function_value = value;
  if (funct_item.attribute_name == "")
    return false;

  rosplan_knowledge_msgs::KnowledgeUpdateService function_instance;
  function_instance.request.update_type = update_type;
  function_instance.request.knowledge = funct_item;

  return ku_client_.call(function_instance);
}

bool KMSClient::add_function(std::string metric_fact, float value)
{
  if (!update_function_kb(
      metric_fact,
      value,
      rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE
    ))
    ROS_ERROR("Could not add function");
}

bool KMSClient::rm_function(std::string metric_fact, float value)
{
  if (!update_function_kb(
      metric_fact,
      value,
      rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE
    ))
    ROS_ERROR("Could not remove function");
}



bool KMSClient::rm_function_regex(std::regex re)
{
  std::vector<std::pair<std::string,float>> current_functions = getCurrentFunctions();
  for (int i = 0; i < current_functions.size(); i++)
    if (std::regex_match(current_functions[i].first, re))
      rm_function(current_functions[i].first, current_functions[i].second);
}

bool KMSClient::update_function(std::string function, float value)
{
  std::vector<std::string> tokens = tokenize(function);
  rosplan_knowledge_msgs::GetAttributeService srv;
  srv.request.predicate_name = tokens[0];
  bool function_updated = false;
  if (!kf_client_.call(srv))
  {
    ROS_ERROR("Could not get functions");
    return false;
  }
  else
  {
    for (int i = 0; i < srv.response.attributes.size(); i++)
    {
      int count = 0;
      for (int j = 0; j < srv.response.attributes[i].values.size(); j++)
      {
        if(srv.response.attributes[i].values[j].value == tokens[j+1])
          count++;
      }
      if (count == srv.response.attributes[i].values.size())
      {
        rm_function(function, srv.response.attributes[i].function_value);
        add_function(function, value);
        function_updated = true;
        break;
      }
    }

    if (!function_updated)
      add_function(function, value);
  }
  return true;
}

bool KMSClient::remove_current_goal()
{
  bool ret = true;

  rosplan_knowledge_msgs::GetAttributeService srv;
  srv.request.predicate_name = "";

  if (!cg_client_.call(srv))
  {
    ROS_ERROR("Could not get the current goal");
    return false;
  }
  else
  {
    for (int i = 0; i < srv.response.attributes.size(); i++)
    {
      rosplan_knowledge_msgs::KnowledgeItem ki = srv.response.attributes[i];
      rosplan_knowledge_msgs::KnowledgeUpdateService rm_goal_instance;
      rm_goal_instance.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
      rm_goal_instance.request.knowledge = ki;

      if (!ku_client_.call(rm_goal_instance))
      {
        ROS_ERROR("Could not remove the goal [%s]", srv.response.attributes[0].attribute_name.c_str());
        ret = false;
      }
      else
        ROS_INFO("Goal [%s] removed", srv.response.attributes[0].attribute_name.c_str());
    }

    return ret;
  }
}

std::vector<std::pair<std::string,float>> KMSClient::getCurrentFunctions()
{
  std::vector<std::pair<std::string,float>> ret;
  std::pair<std::string,float> funct;
  rosplan_knowledge_msgs::GetAttributeService query_fact_instance;
  query_fact_instance.request.predicate_name = "";

  if (!kf_client_.call(query_fact_instance))
  {
    ROS_ERROR("Could not query current functions");
    return ret;
  }

  std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator it;
  for (it = query_fact_instance.response.attributes.begin(); it != query_fact_instance.response.attributes.end(); ++it)
  {
    funct.first = PredicateToString(*it);
    funct.second = it->function_value;
    ret.push_back(funct);
  }

  return ret;
}

std::vector<std::string> KMSClient::getCurrentPredicates()
{
  std::vector<std::string> ret;
  rosplan_knowledge_msgs::GetAttributeService query_fact_instance;
  query_fact_instance.request.predicate_name = "";

  if (!kq_client_.call(query_fact_instance))
  {
    ROS_ERROR("Could not query current predicates");
    return ret;
  }

  std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator it;
  for (it = query_fact_instance.response.attributes.begin(); it != query_fact_instance.response.attributes.end(); ++it)
    ret.push_back(PredicateToString(*it));

  return ret;
}

bool KMSClient::remove_predicate(const std::string& predicate)
{
  rosplan_knowledge_msgs::KnowledgeUpdateService rm_fact_instance;
  rm_fact_instance.request.update_type = 2;  // rosplan_knowledge_msgs::KnowledgeUpdateService::REMOVE_KNOWLEDGE;
  rm_fact_instance.request.knowledge = StringToPredicate(predicate);

  if (!ku_client_.call(rm_fact_instance))
  {
    ROS_ERROR("Could not remove the predicate [%s]", predicate.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Predicate [%s] removed", predicate.c_str());
    return true;
  }
}

bool KMSClient::remove_predicates_regex(std::regex re)
{
  std::vector<std::string> current_predicates = getCurrentPredicates();

  for (int i = 0; i < current_predicates.size(); i++)
    if (std::regex_match(current_predicates[i], re))
      remove_predicate(current_predicates[i]);
}

std::vector<std::string> KMSClient::search_predicates_regex(std::regex re)
{
  std::vector<std::string> ret;
  std::vector<std::string> current_predicates = getCurrentPredicates();

  for (int i = 0; i < current_predicates.size(); i++)
    if (std::regex_match(current_predicates[i], re))
      ret.push_back(current_predicates[i]);

  return ret;
}

bool KMSClient::add_predicate(std::string predicate)
{
  rosplan_knowledge_msgs::KnowledgeItem add_fact_v = StringToPredicate(predicate);

  rosplan_knowledge_msgs::KnowledgeUpdateService add_fact_instance;
  add_fact_instance.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
  add_fact_instance.request.knowledge = add_fact_v;

  if (!ku_client_.call(add_fact_instance))
  {
    ROS_ERROR("Could not add the fact [%s]", predicate.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Predicate [%s] added", predicate.c_str());
    return true;
  }
}

std::vector<std::string> KMSClient::get_instances(const std::string& instance_type)
{
  std::vector<std::string> ret;
  rosplan_knowledge_msgs::GetInstanceService get_instance_srv;

  get_instance_srv.request.type_name = instance_type;

  if (!ki_client_.call(get_instance_srv))
  {
    ROS_ERROR("Could not get instances of type [%s]", instance_type.c_str());
  }
  else

    return get_instance_srv.response.instances;
}
};  // namespace bica_planning
