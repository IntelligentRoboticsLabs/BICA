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
 * Activation.cpp
 *
 *  Created on: 12/05/2016
 *      Author: paco
 */

#include <bica/Activation.h>

#include <string>
#include <vector>

namespace bica
{
Activation::Activation() : nh_(), id(""), active(false), alive(false)
{
}

Activation::Activation(std::string id_i, bool active_i, bool alive_i) : id(id_i), active(active_i), alive(alive_i)
{
  sub_ = nh_.subscribe("/" + id + "/active", 1, &Activation::activeCB, this);
  last_heartbeat_ = ros::Time::now();
}

Activation::Activation(const Activation& other)
{
  id = other.id;
  active = other.active;
  alive = other.alive;

  sub_ = nh_.subscribe("/" + id + "/active", 1, &Activation::activeCB, this);
  last_heartbeat_ = ros::Time::now();
}

Activation::~Activation()
{
}

void Activation::checkAlive()
{
  std::vector<std::string> nodes;
  if (!ros::master::getNodes(nodes))
    ROS_WARN("failed to get list of nodes");

  std::vector<std::string>::iterator nodeit = std::find(nodes.begin(), nodes.end(), "/" + id);

  bool check_hb = ((ros::Time::now() - last_heartbeat_).toSec() > DEAD_T);
  alive = !(nodeit == nodes.end());
}

void Activation::activeCB(const std_msgs::Empty::ConstPtr& msg)
{
  last_heartbeat_ = ros::Time::now();
}

} /* namespace bica */
