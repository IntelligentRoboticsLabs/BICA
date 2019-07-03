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

#include "ros/ros.h"

#include "bica_planning/Executor.h"

class DemoExecutor: public bica_planning::Executor
{
public:

  static const int STATE_INIT = 0;
  static const int STATE_A = 1;
  static const int STATE_B = 2;
  static const int STATE_C = 3;
  static const int STATE_D = 4;

  DemoExecutor()
  {
    init_knowledge();

    ts_ = ros::Time::now();
  }

  void update()
  {
    ros::Time now = ros::Time::now();
    switch(state_)
    {
      case STATE_INIT:
        if ( (now - ts_).toSec() > 5)
        {
          ts_ = now;
          state_ = STATE_A;
          remove_predicate("robot_at leia bedroom");
          ROS_INFO("removing predicate robot_at leia bedroom");
          //remove_instance("robot", "leia");
          //ROS_INFO("removing leia");
        }
        break;
      case STATE_A:
        if ( (now - ts_).toSec() > 5)
        {
          ts_ = now;
          state_ = STATE_B;
          add_instance("robot", "leia");
          ROS_INFO("adding leia");
          add_instance("person", "Anne");
          ROS_INFO("adding Anne");
          add_predicate("robot_at leia bedroom");
          ROS_INFO("add predicate robot_at leia bedroom");

        }
        break;
      case STATE_B:
        if ( (now - ts_).toSec() > 5)
        {
          ts_ = now;
          state_ = STATE_C;
          remove_instance("person", "Anne");
          ROS_INFO("removing Anne");
        }
        break;
      case STATE_C:
        if ( (now - ts_).toSec() > 5)
        {
          ts_ = now;
          state_ = STATE_D;
          remove_predicate("robot_at leia bedroom");
          ROS_INFO("removing predicate robot_at leia bedroom");
          remove_predicate("person_at Jack kitchen");
          ROS_INFO("removing predicate robot_at leia bedroom");
        }
        break;
      case STATE_D:
        if ( (now - ts_).toSec() > 5)
        {
          ts_ = now;
          state_ = STATE_INIT;
          add_predicate("robot_at leia bedroom");
          ROS_INFO("removing predicate robot_at leia bedroom");
          add_predicate("person_at Jack kitchen");
          ROS_INFO("removing predicate person_at Jack kitchen");
        }
        break;

    };

    //call_planner();
  }

private:
  void init_knowledge()
  {
    ROS_INFO("[DemoExecutor] Initializing knowledge-------------------------------------------");

    if (!add_instance("robot", "leia"))
      ROS_ERROR("[DemoExecutor] Problem inserting instance [leia]");

    if (!add_instance("person", "Jack"))
      ROS_ERROR("[DemoExecutor] Problem inserting instance [Jack]");

    if (!add_instance("message", "m1"))
      ROS_ERROR("[DemoExecutor] Problem inserting instance [m1]");

    if (!add_instance("room", "bedroom"))
      ROS_ERROR("[DemoExecutor] Problem inserting instance [bedroom]");

    if (!add_instance("room", "kitchen"))
      ROS_ERROR("[DemoExecutor] Problem inserting instance [kitchen]");

    if (!add_predicate("robot_at leia bedroom"))
      ROS_ERROR("[DemoExecutor] Problem inserting predicate [robot_at leia bedroom]");

    if (!add_predicate("person_at Jack kitchen"))
      ROS_ERROR("[DemoExecutor] Problem inserting predicate [person_at Jack kitchen]");
  }

  ros::Time ts_;
  int state_ = STATE_INIT;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "planning_example_elements_node");
  ros::NodeHandle n;

  DemoExecutor demo_exe;

  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    demo_exe.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

}
