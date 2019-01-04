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
/*
 * Component.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <bica/Component.h>

#include <string>
#include <list>

namespace bica
{
Component::Component() : nh_(), active_(false), root_(true)
{
  activation_srv_ = nh_.advertiseService(ros::this_node::getName() + "/activate", &Component::activateCallback, this);
  deactivation_srv_ =
      nh_.advertiseService(ros::this_node::getName() + "/deactivate", &Component::deActivateCallback, this);

  alive_pub_ = nh_.advertise<std_msgs::Empty>(ros::this_node::getName() + "/active", 1, false);
}

Component::~Component()
{
  ROS_DEBUG("Killed destructor");
}

void Component::setActive(bool act)
{
  if (act)
  {
    activateCode();
    ROS_DEBUG("[%s] Activation", ros::this_node::getName().c_str());
  }
  else
  {
    if (root_)
      deActivateCode();
    ROS_DEBUG("[%s] Deactivation", ros::this_node::getName().c_str());
  }
  active_ = act;
}

bool Component::activateCallback(ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& call)
{
  ROS_INFO("[%s] start from [%s]", ros::this_node::getName().c_str(), call.getCallerName().c_str());
  root_ = false;
  std::string caller = call.getCallerName().substr(1, call.getCallerName().length());

  std::list<Activation>::iterator it = activations_.begin();
  while ((it != activations_.end()) && (it->id != caller))
    ++it;

  if (activations_.empty() || it == activations_.end())
  {
    Activation newact(caller, true, true);

    if (!active_)
      activateCode();

    active_ = true;
    activations_.push_back(newact);

    ROS_INFO("[%s] Added an activation from [%s]", ros::this_node::getName().c_str(), caller.c_str());
  }
  else
    ROS_WARN("[%s] multiple activations from [%s]", ros::this_node::getName().c_str(), caller.c_str());

  return true;
}

bool Component::deActivateCallback(ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& call)
{
  ROS_DEBUG("[%s] stop from [%s]", ros::this_node::getName().c_str(), call.getCallerName().c_str());

  std::string caller = call.getCallerName().substr(1, call.getCallerName().length());

  std::list<Activation>::iterator it = activations_.begin();
  while (it != activations_.end() && (it->id != caller))
    ++it;

  if (it != activations_.end())
  {
    ROS_DEBUG("[%s] removing activation [%s]", ros::this_node::getName().c_str(), caller.c_str());
    it = activations_.erase(it);
  }
  else
    ROS_ERROR("[%s] from a non activator [%s]", ros::this_node::getName().c_str(), caller.c_str());

  if (activations_.empty())
  {
    active_ = false;
    deActivateCode();
  }

  return true;
}

void Component::activateCode()
{
  ROS_INFO("[%s] start code", ros::this_node::getName().c_str());
}

void Component::deActivateCode()
{
  ROS_INFO("[%s] Component stop code", ros::this_node::getName().c_str());
}

void Component::activate(const std::string& id)
{
  std::string caller = ros::this_node::getName();

  if (id == caller)
    return;

  if (!isDependency(id))
  {
    Dependency newdep;
    newdep.id = id;
    newdep.activated = false;
  }
  else
    ROS_WARN("[%s] trying to activate [%s], which is not a dependency. Add it first.",
             ros::this_node::getName().c_str(), id.c_str());
}

void Component::deActivate(const std::string& id)
{
  std::string caller = ros::this_node::getName();

  std::string srv_name = "/" + id + "/deactivate";

  std::list<Dependency>::iterator it = dependencies_.begin();
  while (it != dependencies_.end() && it->id != id)
    ++it;
  if (it != dependencies_.end())
  {
    ros::ServiceClient auxclient = nh_.serviceClient<std_srvs::Empty>(srv_name);
    std_srvs::Empty srv;

    it->activated = !auxclient.call(srv);
  }
}

bool Component::isDependency(const std::string& dep)
{
  ROS_DEBUG("[%s] checking for dependency in %zu", ros::this_node::getName().c_str(), dependencies_.size());

  if (dependencies_.empty())
    return false;

  std::list<Dependency>::iterator it = dependencies_.begin();
  ROS_DEBUG("\tF [%s][%s]", it->id.c_str(), dep.c_str());

  while (it != dependencies_.end())
  {
    bool val = it->id == dep;
    if (val)
    {
      ROS_DEBUG("[%s] == [%s]", it->id.c_str(), dep.c_str());
      break;
    }
    else
    {
      ROS_DEBUG("[%s] != [%s]", it->id.c_str(), dep.c_str());
      ++it;
    }
  }

  return it != dependencies_.end();
}

void Component::addDependency(const Dependency& dep)
{
  if (!isDependency(dep.id))
  {
    ROS_DEBUG("[%s] adding dependecy [%s]", ros::this_node::getName().c_str(), dep.id.c_str());
    dependencies_.push_back(dep);
  }
  else
    ROS_WARN("[%s] NOT adding dependecy [%s]", ros::this_node::getName().c_str(), dep.id.c_str());
}

void Component::addDependency(const std::string& dep)
{
  Dependency newdep(dep, false);
  addDependency(newdep);
}

void Component::removeDependency(const std::string& dep)
{
  Dependency remdep(dep, false);
  removeDependency(remdep);
}

void Component::removeDependency(const Dependency& dep)
{
  ROS_DEBUG("[%s] removing dependency [%s]", ros::this_node::getName().c_str(), dep.id.c_str());

  std::list<Dependency>::iterator it = std::find(dependencies_.begin(), dependencies_.end(), dep);
  if (it != dependencies_.end())
  {
    if (it->alive)
    {
      ROS_DEBUG("[%s] is alive", ros::this_node::getName().c_str());
      deActivate(dep.id);
    }
    else
      ROS_DEBUG("[%s] is not alive", ros::this_node::getName().c_str());

    ROS_DEBUG("[%s] is found", ros::this_node::getName().c_str());
    it = dependencies_.erase(it);
  }
  else
    ROS_DEBUG("[%s] is not found dependency [%s]", ros::this_node::getName().c_str(), dep.id.c_str());
}

void Component::activateDependencies()
{
  if (!active_)
    return;

  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end(); ++it)
  {
    ROS_DEBUG("[%s] dep [%s]", ros::this_node::getName().c_str(), it->id.c_str());

    if (!it->activated && it->alive && it->id != ros::this_node::getName())
    {
      std::string srv_name = "/" + it->id + "/activate";
      ros::ServiceClient auxclient = nh_.serviceClient<std_srvs::Empty>(srv_name);
      std_srvs::Empty srv;

      it->activated = auxclient.call(srv);
      if (it->activated)
        ROS_DEBUG("[%s] activated [%s]", ros::this_node::getName().c_str(), it->id.c_str());
      else
        ROS_ERROR("[%s] Failed to activate [%s]", ros::this_node::getName().c_str(), it->id.c_str());
    }

    if (it->id == ros::this_node::getName())
      it->activated = true;
  }
}

void Component::deActivateDependencies()
{
  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end(); ++it)
  {
    if (it->activated && it->alive && it->id != ros::this_node::getName())
    {
      std::string srv_name = "/" + it->id + "/deactivate";
      ros::ServiceClient auxclient = nh_.serviceClient<std_srvs::Empty>(srv_name);
      std_srvs::Empty srv;

      it->activated = !auxclient.call(srv);
      if (it->activated)
        ROS_DEBUG("[%s] deactivated [%s]", ros::this_node::getName().c_str(), it->id.c_str());
      // else
      // ROS_ERROR( "[%s] Failed to deactivate [%s]", ros::this_node::getName().c_str(), it->id.c_str());

      if (it->id == ros::this_node::getName())
        it->activated = false;
    }
  }
}

void Component::printStatus()
{
  ROS_DEBUG("[%s] Dependencies", ros::this_node::getName().c_str());

  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end(); ++it)
    ROS_DEBUG("\t[%s] activated: %s\talive: %s", it->id.c_str(), it->activated ? "true" : "false",
              it->alive ? "true" : "false");
  ROS_DEBUG("[%s] Activations", ros::this_node::getName().c_str());
  for (std::list<Activation>::iterator it = activations_.begin(); it != activations_.end(); ++it)
    ROS_DEBUG("\t[%s] active: %s\talive: %s", it->id.c_str(), it->active ? "true" : "false", it->alive ? "true" : "fals"
                                                                                                                  "e");
}

void Component::checkDependenciesAlive()
{
  if (!active_)
    return;

  for (std::list<Dependency>::iterator it = dependencies_.begin(); it != dependencies_.end(); ++it)
  {
    it->checkAlive();
    if (!it->alive)
      it->activated = false;
  }
}

void Component::checkActivatorsAlive()
{
  if (!active_)
    return;

  std::list<Activation>::iterator it = activations_.begin();

  while (it != activations_.end())
  {
    it->checkAlive();
    if (!it->alive)
    {
      it = activations_.erase(it);
    }
    else
      ++it;
  }

  if (activations_.empty())
  {
    active_ = false;
    deActivateCode();
  }
}

bool Component::ok()
{
  // printStatus();
  if (!root_)
    checkActivatorsAlive();
  checkDependenciesAlive();

  if (active_)
  {
    ROS_DEBUG("[%s] Active", ros::this_node::getName().c_str());
    activateDependencies();

    step();

    alive_pub_.publish(std_msgs::Empty());
  }
  else
  {
    ROS_DEBUG("[%s] Not active", ros::this_node::getName().c_str());
    deActivateDependencies();
  }

  return ros::ok();
}

} /* namespace bica */
