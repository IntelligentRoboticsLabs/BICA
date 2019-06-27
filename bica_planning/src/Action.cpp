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
#include <bica_planning/Action.h>

#include <string>

namespace bica_planning
{
Action::Action(std::string id, float freq) : id_(id), freq_(freq)
{
  setRoot();
}

bool Action::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
  finished_ = false;
  success_ = true;
  last_msg_ = *msg;
  if (!checkAtStartConditions(msg))
  {
    ROS_ERROR("[%s] AT Start not meet: cancelling", id_.c_str());
    return false;
  }
  setActive(true);
  bool ret = true;
  bool finished = false;
  ros::Rate rate(freq_);
  do
  {
    if (!checkOverAllConditions(msg))
    {
      ROS_ERROR("ALL OVER not meet: cancelling");
      setFail();
    }

    ros::spinOnce();
    rate.sleep();
  } while (ok() && !finished_);

  setActive(false);

  if (success_ && !checkAtEndConditions(msg))
  {
    ROS_ERROR("[%s] AT End not meet: cancelling", id_.c_str());
    return false;
  }

  if (success_)
  {
    ROS_INFO("Action achieved ====================");
  }
  return success_;
}
};  // namespace bica_planning
