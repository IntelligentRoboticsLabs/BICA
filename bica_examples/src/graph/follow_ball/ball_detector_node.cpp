
#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include "bica_graph/graph_client.h"

#include <bica/Component.h>


class BallDetector: public bica::Component
{
public:
	BallDetector()
	: nh_(),
		new_image_(false),
		pcrgb_(new pcl::PointCloud<pcl::PointXYZRGB>),
		tfListener_(tfBuffer_)
	{
		cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &BallDetector::cloudCB, this);

		graph_.add_node("world", "abstract");
		graph_.add_node("leia", "robot");
	  graph_.add_node("ball", "object");

		ROS_INFO("[%s] inited", ros::this_node::getName().c_str());
	}

	void activateCode()
	{
		graph_.add_edge("leia", "wants_see", "ball");
	}

	void deActivateCode()
	{
		graph_.remove_edge("leia", "sees", "ball");
		graph_.remove_edge("leia", "wants_see", "ball");
		graph_.remove_tf_edge("leia", "ball");
	}

	void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
		geometry_msgs::TransformStamped transformStamped;
		sensor_msgs::PointCloud2 cloud_in_bf;

		try{
			transformStamped = tfBuffer_.lookupTransform("base_footprint", cloud_in->header.frame_id, ros::Time(0));
			tf2::doTransform(*cloud_in, cloud_in_bf, transformStamped);

			pcl::fromROSMsg(cloud_in_bf, *pcrgb_);
			new_image_ = true;
		}
		catch (tf2::TransformException& ex)
  	{
    	ROS_WARN("%s", ex.what());
  	}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterBall()
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		for(it=pcrgb_->begin(); it!=pcrgb_->end(); ++it)
		{
			if(!std::isnan(it->x))
			{
				pcl::PointXYZHSV hsv;
				pcl::PointXYZRGBtoXYZHSV(*it, hsv);

				pcl::PointXYZHSV hsv_scaled;
				hsv_scaled.s = hsv.s * 100.0;
				hsv_scaled.v = hsv.v * 100.0;
				hsv_scaled.h = hsv.h;

				if( hsv_scaled.h > 0 &&  hsv_scaled.h < 80 &&
						hsv_scaled.s > 90 &&  hsv_scaled.s < 105 &&
						hsv_scaled.v > 36 &&  hsv_scaled.v < 67)
				{
					pcrgb_out->push_back(*it);
				}

			}
		}

		return pcrgb_out;
	}

	tf2::Transform getTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
	{

			float x, y, z;
			int c = 0;
			x = x = z = 0.0;

			pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
			for(it=cloud_filtered->begin(); it!=cloud_filtered->end(); ++it)
			{
				if(!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
				{
					x += it->x;
					y += it->y;
					z += it->z;
					c++;
				}
			}

			if(c!=0)
			{
				x = x/c;
				y = y/c;
				z = z/c;
			}

			tf2::Transform transform;
			transform.setOrigin(tf2::Vector3(x, y, z));
			transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

			ROS_INFO("Detected in (%lf, %lf)", x, y);

			return transform;
	}


	void step()
	{
		if (!isActive()) return;


		ROS_INFO("[%s] step", ros::this_node::getName().c_str());

		if (new_image_)
		{
			new_image_ = false;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered = filterBall();

			if (pc_filtered->size() > 0)
			{
				graph_.remove_edge("leia", "wants_see", "ball");
				graph_.add_edge("leia", "sees", "ball");
				graph_.add_edge("leia", getTransform(pc_filtered), "ball");
			}
			else
			{
				graph_.add_edge("leia", "wants_see", "ball");
				graph_.remove_edge("leia", "sees", "ball");
				graph_.remove_tf_edge("leia", "ball");
			}
		}
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_;
	tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
	tf2_ros::TransformBroadcaster tfBroadcaster_;

	bica_graph::GraphClient graph_;

	bool new_image_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_detector");

	BallDetector ball_detector;

	ros::Rate loop_rate(5);
	while(ball_detector.ok())
	{
		ball_detector.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
