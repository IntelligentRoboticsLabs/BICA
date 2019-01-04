#include <ros/ros.h>

#include "test_M.h"


class test_M_impl: public bica::test_M
{

  void State_B_code_once()
  {
    ROS_INFO("[%s] State B ", ros::this_node::getName().c_str());
  }

  void State_C_code_iterative()
  {
    ROS_INFO("[%s] State C ", ros::this_node::getName().c_str());
  }

  void State_A_code_once()
  {
    ROS_INFO("[%s] State A ", ros::this_node::getName().c_str());
  }

  bool State_A_2_State_B()
  {
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
  }

  bool State_B_2_State_C()
  {
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
  }

  bool State_C_2_State_A()
  {
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
  }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_M");

	test_M_impl test_m;

	ros::Rate loop_rate(7);
	while(test_m.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
