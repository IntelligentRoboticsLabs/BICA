/*
 * test_b_node.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <bica/Component.h>

class TestB: public bica::Component
{
public:
	TestB()
	{
	}

	~TestB()
	{
	}

	void step()
	{
		if(!isActive()) return;

		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_B");

	TestB test_b;

	ros::Rate loop_rate(7);
	while(test_b.ok())
	{
		test_b.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
