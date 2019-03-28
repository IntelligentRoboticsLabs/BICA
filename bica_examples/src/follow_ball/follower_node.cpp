
#include <ros/ros.h>

#include <bica/Component.h>

class Follower: public bica::Component
{
public:
	Follower()
	{

	}

	~Follower()
	{
	}

	void step()
	{
		if (!isActive()) return;

		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_follower");

	Follower follower;

	ros::Rate loop_rate(5);
	while(follower.ok())
	{
		follower.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
