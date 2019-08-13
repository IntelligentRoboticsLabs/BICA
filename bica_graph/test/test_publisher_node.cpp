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

#include <ros/ros.h>

#include <bica_graph/graph.h>
#include <bica_graph/graph_client.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "graph_publisher_node");
  ros::NodeHandle n;

  ROS_INFO("Creating graph client...");
  bica_graph::GraphClient client;
  ROS_INFO("done");
  ros::Rate rate(5);
  ros::Time start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 2.0 )
  {
    ROS_INFO("spinnging 1");
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("adding node 1");
  client.add_node("leia", "robot");
  ROS_INFO("adding node 2");
  client.add_node("bedroom", "room");
  ROS_INFO("adding edge is");
  client.add_edge("leia", "is", "bedroom");

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 200.0 )
  {
    ROS_INFO("adding transform spinnging");

    tf2::Transform tf_r2l(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3((ros::Time::now() - start).toSec(), 0, 0));

    client.add_edge("bedroom", tf_r2l, "leia");

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
