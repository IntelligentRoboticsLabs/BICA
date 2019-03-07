#include <ros/ros.h>

#include <bica_graph/graph.h>
#include <bica_graph/graph_publisher.h>
#include <bica_graph/tf_relation.h>

class GraphServer
{
public:
  GraphServer()
  : nh_(), graph_(new bica_graph::BicaGraph()), graph_pub_(nh_, graph_)
  {
  }

  void init()
  {
    auto node = graph_->create_node("leia", "robot");
    auto bedroom =  graph_->create_node("bedroom", "room");
    auto relation1 = node->add_relation("is", bedroom);
    auto relation2 = node->add_relation("isbis", bedroom);

    geometry_msgs::TransformStamped tf;
    tf.header.seq = 1;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "/world";
    tf.child_frame_id = "/robot";
    tf.transform.translation.x = 1;
    tf.transform.translation.y = 1;
    tf.transform.translation.z = 1;
    tf.transform.rotation.x = 0;
    tf.transform.rotation.y = 0;
    tf.transform.rotation.z = 0;
    tf.transform.rotation.w = 1;

    auto relation_tf = node->add_tf_relation(tf, bedroom);

    graph_pub_.send_graph();
  }

  void update()
  {
    auto node = graph_->get_node("leia");

    for (auto relation: node->get_relations())
    {
      if (relation->get_type() == "tf")
      {
        auto r = std::dynamic_pointer_cast<bica_graph::TFRelation>(relation);
        geometry_msgs::TransformStamped& tf = r->get_tf();
        tf.header.stamp = ros::Time::now();
      }
    }

    graph_pub_.send_graph();
  }

private:
  ros::NodeHandle nh_;
  bica_graph::BicaGraph::SharedPtr graph_;
  bica_graph::GraphPublisher graph_pub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_server_node");
  ros::NodeHandle n;

  GraphServer server;
  server.init();

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    server.update();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
