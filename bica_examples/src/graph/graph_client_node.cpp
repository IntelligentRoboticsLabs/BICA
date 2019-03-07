#include <ros/ros.h>

#include <bica_graph/graph.h>
#include <bica_graph/graph_listener.h>
#include <bica_graph/tf_relation.h>

class GraphClient
{
public:
  GraphClient()
  : nh_(), graph_(new bica_graph::BicaGraph()), graph_list_(nh_, graph_)
  {
  }


  void update()
  {
    std::cout<<*graph_<<std::endl;
  }

private:
  ros::NodeHandle nh_;
  bica_graph::BicaGraph::SharedPtr graph_;
  bica_graph::GraphListener graph_list_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_client_node");
  ros::NodeHandle n;

  GraphClient client;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    client.update();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
