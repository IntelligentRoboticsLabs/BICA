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
  ros::init(argc, argv, "graph_subscriber_node");
  ros::NodeHandle n;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf_listener(tfBuffer);

  ROS_INFO("Creating graph client...");
  bica_graph::GraphClient client;
  ROS_INFO("done");
  ros::Rate rate(5);
  ros::Time start;

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 5.0 )
  {
    ros::spinOnce();
    rate.sleep();
  }

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 200.0 )
  {
    if (client.exist_tf_edge("bedroom", "leia"))
    {
      tf2::Transform tf = client.get_tf_edge("bedroom", "leia").get();
      ROS_INFO("Reading (%lf, %lf, %lf)", tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());
    }

    std::string error;
    if (tfBuffer.canTransform("leia", "bedroom", ros::Time(0), ros::Duration(0.1), &error))
    {
      geometry_msgs::TransformStamped tf_msg;
      tf2::Stamped<tf2::Transform> tf2;
      tf_msg = tfBuffer.lookupTransform("bedroom", "leia", ros::Time(0));
      tf2::fromMsg(tf_msg, tf2);

      ROS_INFO("Reading msg (%lf, %lf, %lf)",
        tf_msg.transform.translation.x, tf_msg.transform.translation.y,tf_msg.transform.translation.z);
    }
    else
    {
      ROS_ERROR("Can't transform: %s", error.c_str());
    }

    tf2::Stamped<tf2::Transform> tf2 = client.get_tf("bedroom", "leia");
    ROS_INFO("Reading 2 (%lf, %lf, %lf)", tf2.getOrigin().x(), tf2.getOrigin().y(), tf2.getOrigin().z());

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
