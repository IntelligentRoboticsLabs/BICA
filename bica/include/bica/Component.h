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
 * Component.h
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#ifndef BICA_COMPONENT_H
#define BICA_COMPONENT_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/service_callback_helper.h>

#include <std_srvs/Empty.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

#include <bica/Dependency.h>
#include <bica/Activation.h>

#include <std_msgs/Empty.h>

#include <string>
#include <list>

namespace bica
{
class Component
{
public:
  Component();
  virtual ~Component();

  void setRoot()
  {
    root_ = true;
  }
  bool isActive()
  {
    return active_;
  };
  void setActive(bool act = true);

  virtual void step() {}

  virtual bool ok();

protected:
  virtual void activateCode();
  virtual void deActivateCode();

  void activate(const std::string& id);
  void deActivate(const std::string& id);

  bool isDependency(const std::string& dep);
  void addDependency(const std::string& dep);
  void addDependency(const Dependency& dep);
  void removeDependency(const std::string& dep);
  void removeDependency(const Dependency& dep);

  bool active_;
  bool root_;

  ros::Publisher alive_pub_;

private:
  bool activateCallback(ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& call);
  bool deActivateCallback(ros::ServiceEvent<std_srvs::Empty::Request, std_srvs::Empty::Response>& call);
  void activateDependencies();
  void deActivateDependencies();
  void printStatus();

  void checkActivatorsAlive();
  void checkDependenciesAlive();

  ros::NodeHandle nh_;

  ros::ServiceServer activation_srv_;
  ros::ServiceServer deactivation_srv_;

  std::list<Dependency> dependencies_;
  std::list<Activation> activations_;

  static Component* instance_;
};

}  // namespace bica

#endif  // BICA_COMPONENT_H
