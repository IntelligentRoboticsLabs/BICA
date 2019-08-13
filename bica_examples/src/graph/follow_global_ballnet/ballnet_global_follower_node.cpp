#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bica/Component.h>

#include "bica_graph/graph_client.h"

#include <geometry_msgs/Twist.h>

#include "ball_net_follower.h"

class Follower: public bica::ball_net_follower
{
public:
	Follower()
	:  nh_()
	{
		cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

		graph_.add_node("leia", "robot");
		graph_.add_node("world", "abstract");
		graph_.add_node("ball", "object");
		graph_.add_node("yellow_net", "object");
		graph_.add_node("blue_net", "object");

		graph_.set_tf_identity("base_footprint", "leia");
		graph_.set_tf_identity("odom", "world");
	}

	void Yellow_code_iterative()
	{
		geometry_msgs::Twist cmd;

		if (graph_.exist_tf_edge("world", "yellow_net"))
		{
			try{
				tf2::Stamped<tf2::Transform> w2r = graph_.get_tf("world", "leia");
				tf2::Transform r2w = w2r.inverse();
				tf2::Stamped<tf2::Transform> w2y = graph_.get_tf("world", "yellow_net");
				tf2::Transform r2y = r2w*w2y;

				ROS_INFO("I see the yellow_net in %lf %lf", r2y.getOrigin().x(), r2y.getOrigin().y());
				cmd.angular.z = std::max(std::min(atan2(r2y.getOrigin().y(), r2y.getOrigin().x()), 0.2), -0.2);
				cmd.linear.x = std::min(r2y.getOrigin().length() - 1.5, 0.2);
			}catch(bica_graph::exceptions::TransformNotPossible& e){
				cmd.angular.z = 0.3;
				ROS_INFO("I don't know where is the yellow_net :'(");
			}
		}
		else
		{
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.3;
			ROS_INFO("I don't know where is the yellow_net :'(");
		}

		cmd_pub_.publish(cmd);
	}

	void Blue_code_iterative()
	{
		geometry_msgs::Twist cmd;

		if (graph_.exist_tf_edge("world", "blue_net"))
		{
			try{
				tf2::Stamped<tf2::Transform> w2r = graph_.get_tf("world", "leia");
				tf2::Transform r2w = w2r.inverse();
				tf2::Stamped<tf2::Transform> w2b = graph_.get_tf("world", "blue_net");
				tf2::Transform r2b = r2w*w2b;

				cmd.angular.z = std::max(std::min(atan2(r2b.getOrigin().y(), r2b.getOrigin().x()), 0.2), -0.2);
				cmd.linear.x = std::min(r2b.getOrigin().length() - 1.5, 0.2);
			}catch(bica_graph::exceptions::TransformNotPossible& e){
				cmd.linear.x = 0.0;
				cmd.angular.z = 0.3;
			}
		}else
		{
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.3;
			ROS_INFO("I don't know where is the blue_net :'(");
		}

		cmd_pub_.publish(cmd);
	}

	void Ball_code_iterative()
	{
		if (!isActive()) return;

		geometry_msgs::Twist cmd;
		bool ball_seen;

		tf2::Transform r2b;

		try{

			tf2::Stamped<tf2::Transform> w2r = graph_.get_tf("world", "leia");
			tf2::Transform r2w = w2r.inverse();
			tf2::Stamped<tf2::Transform> w2b = graph_.get_tf("world", "ball");
			r2b = r2w*w2b;

			if (graph_.exist_edge("leia", "sees", "ball"))
			{
				ball_seen = true;
			}
			else
			{
				if ((ros::Time::now() - state_ts_).toSec() < 10)
				{
					ball_seen = true;
				}else if ((ros::Time::now() - w2b.stamp_).toSec() < 1)
				{
					ball_seen = true;
				} else
				{
					ball_seen = false;
				}
			}
		}catch(bica_graph::exceptions::TransformNotPossible& e){
			ball_seen = false;
		}

		if (ball_seen)
		{
			ROS_INFO("I see the ball in %lf %lf", r2b.getOrigin().x(), r2b.getOrigin().y());
			cmd.angular.z = std::max(std::min(atan2(r2b.getOrigin().y(), r2b.getOrigin().x()), 0.2), -0.2);
			cmd.linear.x = std::min(r2b.getOrigin().length() - 1.5, 0.2);
		}else{
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.3;
			ROS_INFO("I don't know where is the ball :'(");
	  }

		cmd_pub_.publish(cmd);
	}


	bool Blue_2_Ball()
	{
		return (ros::Time::now() - state_ts_).toSec() > 20.0;
	}

	bool Ball_2_Yellow()
	{
		return (ros::Time::now() - state_ts_).toSec() > 20.0;
	}

	bool Yellow_2_Blue()
	{
		return (ros::Time::now() - state_ts_).toSec() > 20.0;
	}

private:
	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

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
