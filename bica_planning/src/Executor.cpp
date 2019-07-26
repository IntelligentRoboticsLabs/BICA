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
#include "bica_planning/Executor.h"

namespace bica_planning
{
Executor::Executor()
  : nh_()
  , problem_generator_client_(nh_.serviceClient<std_srvs::Empty>
      ("/rosplan_problem_interface/problem_generation_server"))
  , planning_server_state_client_(nh_.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server"))
  , plan_parser_client_(nh_.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan"))
  , plan_dispatcher_client_(nh_.serviceClient<std_srvs::Empty>("/rosplan_plan_dispatcher/dispatch_plan"))
  , system_state_("Ready")
{
  system_state_sub_ = nh_.subscribe("/rosplan_planner_interface/system_state", 10, &Executor::system_stateCB, this);
}

void Executor::call_planner()
{
  std_srvs::Empty service;
  if (!problem_generator_client_.call(service))
    ROS_ERROR("[Executor] problem_generation_server service responded with an error");
  else
    ROS_INFO("Problem generation success");
  if (!planning_server_state_client_.call(service))
    ROS_ERROR("[Executor] planning_server service responded with an error");
  else
    ROS_INFO("Planning success");
  if (!plan_parser_client_.call(service))
    ROS_ERROR("[Executor] parse_plan service responded with an error");
  else
    ROS_INFO("Plan parse success");
  if (!plan_dispatcher_client_.call(service))
    ROS_ERROR("[Executor] dispatch_plan service responded with an error");
  else
    ROS_INFO("Plan dispatched success");

  ros::spinOnce();  // This call is to avoid sync problems. These calls take a long time
}

void Executor::system_stateCB(const std_msgs::String::ConstPtr& msg)
{
  system_state_ = msg->data;
}

};  // namespace bica_planning
