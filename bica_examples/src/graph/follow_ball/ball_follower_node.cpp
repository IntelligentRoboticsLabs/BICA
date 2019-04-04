
#include <ros/ros.h>

#include <bica/Component.h>
#include <bica_graph/graph_handler.h>

#include <geometry_msgs/Twist.h>

class Follower: public bica::Component
{
public:
	Follower()
	:  nh_(),
		 graph_handler_(nh_)
	{
		cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

		addDependency("ball_detector");
	}

	void step()
	{
		if (!isActive()) return;

		geometry_msgs::Twist cmd;

		if (graph_handler_.contains_relation("leia", "sees", "ball"))
		{
			tf::Transform tf_ball = graph_handler_.get_tf_relation("leia", "ball")->get_transform();

			cmd.angular.z = std::max(std::min(atan2(tf_ball.getOrigin().y(), tf_ball.getOrigin().x()), 0.2), -0.2);
			cmd.linear.x = std::min(tf_ball.getOrigin().length() - 0.6, 0.2);
			
			ROS_INFO("I see the ball in %lf %lf", tf_ball.getOrigin().x(), tf_ball.getOrigin().y());
		}
		else
		{
			cmd.angular.z = 0.3;
			ROS_INFO("I don't see the ball :'(");
		}

		cmd_pub_.publish(cmd);
	}

private:
	ros::NodeHandle nh_;
	bica_graph::GraphHandler graph_handler_;

	ros::Publisher cmd_pub_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_follower");

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
