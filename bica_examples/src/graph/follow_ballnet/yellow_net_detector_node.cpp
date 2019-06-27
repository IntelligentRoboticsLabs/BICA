
#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include "bica_graph/graph_client.h"

#include <bica/Component.h>


class YellowNetDetector: public bica::Component
{
public:
	YellowNetDetector()
	: nh_(),
		new_image_(false),
		pcrgb_(new pcl::PointCloud<pcl::PointXYZRGB>)
	{
		cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &YellowNetDetector::cloudCB, this);

		graph_.add_node("leia", "robot");
	  graph_.add_node("yellow_net", "object");
		graph_.add_edge("leia", std::string("wants_see"), "yellow_net");
	}

	void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
		sensor_msgs::PointCloud2 cloud_in_bf;
		pcl_ros::transformPointCloud(std::string("base_footprint"),
			*cloud_in, cloud_in_bf, tfListener_);

		pcl::fromROSMsg(cloud_in_bf, *pcrgb_);
		new_image_ = true;
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

				if( hsv_scaled.h > 48 &&  hsv_scaled.h < 68 &&
						hsv_scaled.s > 25 &&  hsv_scaled.s < 91 &&
						hsv_scaled.v > 25 &&  hsv_scaled.v < 80)
				{
					pcrgb_out->push_back(*it);
				}

			}
		}

		return pcrgb_out;
	}

	tf::Transform getTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
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

			tf::Transform transform;
			transform.setOrigin(tf::Vector3(x, y, z));
			transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

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
				graph_.remove_edge("leia", "yellow_net", std::string("wants_see"));
				graph_.add_edge("leia", std::string("sees"), "yellow_net");
				graph_.add_edge("leia", getTransform(pc_filtered), "yellow_net");
			}
			else
			{
				graph_.remove_edge("leia", "yellow_net", std::string("sees"));
				graph_.remove_edge<tf::Transform>("leia", "yellow_net");
				graph_.add_edge("leia", std::string("wants_see"), "yellow_net");
			}
		}
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_;
	tf::TransformListener tfListener_;
	tf::TransformBroadcaster tfBroadcaster_;

	bica_graph::GraphClient graph_;

	bool new_image_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "yellow_net_detector");

	YellowNetDetector yellow_net_detector;

	ros::Rate loop_rate(5);
	while(yellow_net_detector.ok())
	{
		yellow_net_detector.step();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}