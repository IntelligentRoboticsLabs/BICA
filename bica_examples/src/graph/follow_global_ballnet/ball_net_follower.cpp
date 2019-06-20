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

*   THIS SOFTWARE IS PROVball_net_followerED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCball_net_followerENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "ball_net_follower.h"

namespace bica
{
ball_net_follower::ball_net_follower() : state_(BALL), myBaseId_("ball_net_follower")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

ball_net_follower::~ball_net_follower()
{
}

void ball_net_follower::activateCode()
{
  	deactivateAllDeps();

	state_ = BALL;
	state_ts_ = ros::Time::now();

	Ball_activateDeps();
	Ball_code_once();

}

bool ball_net_follower::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case YELLOW:

	Yellow_code_iterative();

	msg.data = "Yellow";
	if(Yellow_2_Blue())
	{

	deactivateAllDeps();

	state_ = BLUE;
	state_ts_ = ros::Time::now();

	Blue_activateDeps();
	Blue_code_once();
	}
	state_pub_.publish(msg);
	break;

	case BLUE:

	Blue_code_iterative();

	msg.data = "Blue";
	if(Blue_2_Ball())
	{

	deactivateAllDeps();

	state_ = BALL;
	state_ts_ = ros::Time::now();

	Ball_activateDeps();
	Ball_code_once();
	}
	state_pub_.publish(msg);
	break;

	case BALL:

	Ball_code_iterative();

	msg.data = "Ball";
	if(Ball_2_Yellow())
	{

	deactivateAllDeps();

	state_ = YELLOW;
	state_ts_ = ros::Time::now();

	Yellow_activateDeps();
	Yellow_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
ball_net_follower::deactivateAllDeps()
{
	removeDependency("ball_detector");
	removeDependency("yellow_net_detector");
	removeDependency("blue_net_detector");
};

void
ball_net_follower::Yellow_activateDeps()
{
	addDependency("yellow_net_detector");
}

void
ball_net_follower::Blue_activateDeps()
{
	addDependency("blue_net_detector");
}

void
ball_net_follower::Ball_activateDeps()
{
	addDependency("ball_detector");
}



} /* namespace bica */
