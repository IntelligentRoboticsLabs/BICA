
#include <ros/ros.h>

#include <bica/Component.h>
#include <bica_graph/graph_handler.h>

#include <geometry_msgs/Twist.h>

#include "ball_net_follower.h"

class Follower: public bica::ball_net_follower
{
public:
	Follower()
	:  nh_(),
		 graph_handler_(nh_)
	{
		cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
	}

	void Yellow_code_iterative()
	{
		geometry_msgs::Twist cmd;

		if (graph_handler_.contains_relation("leia", "sees", "yellow_net"))
		{
			tf::Transform tf_yellow_net = graph_handler_.get_tf_relation("leia", "yellow_net")->get_transform();

			cmd.angular.z = std::max(std::min(atan2(tf_yellow_net.getOrigin().y(), tf_yellow_net.getOrigin().x()), 0.2), -0.2);
			cmd.linear.x = std::min(tf_yellow_net.getOrigin().length() - 1.5, 0.2);

			ROS_INFO("I see the yellow_net in %lf %lf", tf_yellow_net.getOrigin().x(), tf_yellow_net.getOrigin().y());
		}
		else
		{
			cmd.angular.z = 0.3;
			ROS_INFO("I don't see the yellow_net :'(");
		}

		cmd_pub_.publish(cmd);
	}

	void Blue_code_iterative()
	{
		geometry_msgs::Twist cmd;

		if (graph_handler_.contains_relation("leia", "sees", "blue_net"))
		{
			tf::Transform tf_blue_net = graph_handler_.get_tf_relation("leia", "blue_net")->get_transform();

			cmd.angular.z = std::max(std::min(atan2(tf_blue_net.getOrigin().y(), tf_blue_net.getOrigin().x()), 0.2), -0.2);
			cmd.linear.x = std::min(tf_blue_net.getOrigin().length() - 1.5, 0.2);

			ROS_INFO("I see the blue_net in %lf %lf", tf_blue_net.getOrigin().x(), tf_blue_net.getOrigin().y());
		}
		else
		{
			cmd.angular.z = 0.3;
			ROS_INFO("I don't see the blue_net :'(");
		}

		cmd_pub_.publish(cmd);
	}

	void Ball_code_iterative()
	{
		if (!isActive()) return;

		geometry_msgs::Twist cmd;

		if (graph_handler_.contains_relation("leia", "sees", "ball"))
		{
			tf::Transform tf_ball_net = graph_handler_.get_tf_relation("leia", "ball")->get_transform();

			cmd.angular.z = std::max(std::min(atan2(tf_ball_net.getOrigin().y(), tf_ball_net.getOrigin().x()), 0.2), -0.2);
			cmd.linear.x = std::min(tf_ball_net.getOrigin().length() - 1.5, 0.2);

			ROS_INFO("I see the ball in %lf %lf", tf_ball_net.getOrigin().x(), tf_ball_net.getOrigin().y());
		}
		else
		{
			cmd.angular.z = 0.3;
			ROS_INFO("I don't see the ball :'(");
		}

		cmd_pub_.publish(cmd);
	}


	bool Blue_2_Ball()
	{
		return (ros::Time::now() - state_ts_).toSec() > 30.0;
	}

	bool Ball_2_Yellow()
	{
		return (ros::Time::now() - state_ts_).toSec() > 30.0;
	}

	bool Yellow_2_Blue()
	{
		return (ros::Time::now() - state_ts_).toSec() > 30.0;
	}

private:
	ros::NodeHandle nh_;
	bica_graph::GraphHandler graph_handler_;

	ros::Publisher cmd_pub_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ballnet_follower");

	Follower follower;

	ros::Rate loop_rate(5);
	while(follower.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
