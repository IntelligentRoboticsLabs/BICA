/*
 * test_b_node.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <bica/Component.h>

class TestA: public bica::Component
{
public:
	TestA()
	{
    addDependency("node_B");
    addDependency("node_C");
	}

	~TestA()
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
	ros::init(argc, argv, "node_A");

	TestA test_a;

	ros::Rate loop_rate(7);
	while(test_a.ok())
	{
		test_a.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
