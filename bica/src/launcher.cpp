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
 * test_a_node.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>

#include <bica/Component.h>

#include <random>
#include <string>

class Launcher : public bica::Component
{
public:
  explicit Launcher(std::string dep) : nh_("~")
  {
    addDependency(dep);
  }

  void step()
  {
    ROS_INFO("launcher::step");
  }

private:
  ros::NodeHandle nh_;
};

int main(int argc, char** argv)
{
  srand(time(NULL));
  unsigned int seed = time(NULL);

  ros::init(argc, argv, "launcher" + std::to_string(rand_r(&seed) % 1000));
  ros::NodeHandle nh;

  if (argc < 2)
  {
    ROS_ERROR("Usage: launcher component");
    return 1;
  }

  bool any_active = false;

  ROS_INFO("Launching [%s]", argv[1]);
  Launcher launch_comp(argv[1]);

  launch_comp.setRoot();
  launch_comp.setActive(true);

  ros::Rate loop_rate(5);

  while (launch_comp.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
