/*
 * test_b_node.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <bica/Component.h>

class TestC: public bica::Component
{
public:
	TestC()
	{
    addDependency("node_D");
	}

	~TestC()
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
	ros::init(argc, argv, "node_C");

	TestC test_c;

	ros::Rate loop_rate(7);
	while(test_c.ok())
	{
		test_c.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
