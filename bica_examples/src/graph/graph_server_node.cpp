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
    auto node_leia = graph_->create_node("leia", "robot");
    auto node_r2d2 = graph_->create_node("r2d2", "robot");
    auto node_bedroom =  graph_->create_node("bedroom", "room");
    auto node_kitchen =  graph_->create_node("kitchen", "room");
    auto node_bathroom =  graph_->create_node("bathroom", "room");
    auto node_corridor =  graph_->create_node("corridor", "room");
    auto node_apple =  graph_->create_node("apple", "object");
    auto node_jack =  graph_->create_node("Jack", "person");
    auto node_anne =  graph_->create_node("Anne", "person");

    auto leia_at = node_leia->add_relation("is", node_kitchen);
    auto r2d2_at = node_r2d2->add_relation("is", node_bedroom);
    auto jack_at = node_jack->add_relation("is", node_kitchen);
    auto anne_at = node_anne->add_relation("is", node_bathroom);
    auto apple_at = node_apple->add_relation("is", node_kitchen);
    auto leia_sees = node_leia->add_relation("sees", node_apple);
    auto leia_near_jack = node_leia->add_relation("near", node_jack);

    auto bedroom2corridor = node_bedroom->add_relation("connects", node_corridor);
    auto corridor2bedroom = node_corridor->add_relation("connects", node_bedroom);
    auto kitchen2corridor = node_kitchen->add_relation("connects", node_corridor);
    auto corridor2kitchen = node_corridor->add_relation("connects", node_kitchen);
    auto bathroomcorridor = node_bathroom->add_relation("connects", node_corridor);
    auto corridor2bathroom = node_corridor->add_relation("connects", node_bathroom);

    tf::Transform tf_apple(tf::Quaternion(0,0,0,1), tf::Vector3(1,0,1));

    auto leia_apple_tf = node_leia->add_tf_relation(tf_apple, node_apple);

    tf::Transform tf_jack(tf::Quaternion(0,0,0,1), tf::Vector3(4,2,0));

    auto leia_jack_tf = node_leia->add_tf_relation(tf_jack, node_jack);

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
        tf::StampedTransform tf = r->get_transform();
        r->set_transform(tf);
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
