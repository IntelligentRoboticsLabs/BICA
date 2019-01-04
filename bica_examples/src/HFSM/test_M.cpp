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

*   THIS SOFTWARE IS PROVtest_MED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCtest_MENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "test_M.h"

namespace bica
{
test_M::test_M() : state_(STATE_A), myBaseId_("test_M")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

test_M::~test_M()
{
}

void test_M::activateCode()
{
  	deactivateAllDeps();

	state_ = STATE_A;
	state_ts_ = ros::Time::now();

	State_A_activateDeps();
	State_A_code_once();

}

bool test_M::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case STATE_C:

	State_C_code_iterative();

	msg.data = "State_C";
	if(State_C_2_State_A())
	{

	deactivateAllDeps();

	state_ = STATE_A;
	state_ts_ = ros::Time::now();

	State_A_activateDeps();
	State_A_code_once();
	}
	state_pub_.publish(msg);
	break;

	case STATE_A:

	State_A_code_iterative();

	msg.data = "State_A";
	if(State_A_2_State_B())
	{

	deactivateAllDeps();

	state_ = STATE_B;
	state_ts_ = ros::Time::now();

	State_B_activateDeps();
	State_B_code_once();
	}
	state_pub_.publish(msg);
	break;

	case STATE_B:

	State_B_code_iterative();

	msg.data = "State_B";
	if(State_B_2_State_C())
	{

	deactivateAllDeps();

	state_ = STATE_C;
	state_ts_ = ros::Time::now();

	State_C_activateDeps();
	State_C_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
test_M::deactivateAllDeps()
{
	removeDependency("node_A");
	removeDependency("node_B");
	removeDependency("node_C");
};

void
test_M::State_C_activateDeps()
{
	addDependency("node_C");
}

void
test_M::State_A_activateDeps()
{
	addDependency("node_A");
}

void
test_M::State_B_activateDeps()
{
	addDependency("node_B");
}



} /* namespace bica */
