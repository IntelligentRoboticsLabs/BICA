/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Intelligent Robotics Core S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intelligent Robotics Core nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

/* Author: Francisco Martín Rico - fmrico@gmail.com */

#include "ros/ros.h"

#include <bica_graph/graph_client.h>

#include <vector>
#include <list>

std::vector<std::string> tokenize(const std::string& text)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos)
  {
    end = text.find(" ", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}


class GraphTerminal
{
public:
  GraphTerminal() : spinner_(4)
  {
    spinner_.start();
  }

  void run_console()
  {
    std::string line;
    bool success = true;

    std::cout<<"BICA Graph console. Type \"quit\" to finish"<<std::endl;
    std::cout<<"> ";
    while (std::getline(std::cin, line))
    {
      if (line == "quit")
        break;

      process_command(line);
      std::cout<<"> ";
    }

    std::cout<<"Finishing..."<<std::endl;
    ros::shutdown();
  }

  void list_nodes(void)
  {
    const std::list<bica_graph::Node>&nodes = graph_.get_nodes();

    std::cout << "total nodes: " << nodes.size() << std::endl;
    for (auto node : nodes)
    {
      std::cout << "\t[" << node.get_id() << "]\t(type: [" << node.get_type() << "])" << std::endl;
    }
  }

  void list_edges(void)
  {
    const std::list<bica_graph::StringEdge>&string_edges = graph_.get_string_edges();
    const std::list<bica_graph::DoubleEdge>&double_edges = graph_.get_double_edges();
    const std::list<bica_graph::TFEdge>&tf_edges = graph_.get_tf_edges();

    std::cout << "total string edges: " << string_edges.size() << std::endl;
    for (auto edge : string_edges)
    {
      std::cout << "\t(" << edge.get_source() << ") --[" << edge.get()
        << "]--> (" << edge.get_target() << ")" << std::endl;
    }

    std::cout << "total double edges: " << double_edges.size() << std::endl;
    for (auto edge : double_edges)
    {
      std::cout << "\t(" << edge.get_source() << ") --[" << edge.get()
        << "]--> (" << edge.get_target() << ")" << std::endl;
    }

    std::cout << "total tf edges: " << tf_edges.size() << std::endl;
    for (auto edge : tf_edges)
    {
      std::cout << "\t(" << edge.get_source() << ") --["
        << edge.get().getOrigin().x() << " " << edge.get().getOrigin().y() << " "
        << edge.get().getOrigin().z() << "   " << edge.get().getRotation().x() << " "
        << edge.get().getRotation().y() << " " << edge.get().getRotation().z() << " "
        << edge.get().getRotation().w()
        << "]--> (" << edge.get_target() << ")" << std::endl;
    }
  }

  void process_list(const std::vector<std::string>& command)
  {
    if (command.size() == 1)
    {
      list_nodes();
      list_edges();
    }
    else if ((command.size() == 2) && (command[1] == "nodes"))
    {
      list_nodes();
    }
    else if ((command.size() == 2) && (command[1] == "edges"))
    {
      list_edges();
    } else
      std::cout<<"\tUsage: list [nodes|edges]"<<std::endl;
  }

  void process_add(const std::vector<std::string>& command)
  {
    if (command.size() > 1)
    {
      if (command[1] == "node")
      {
        if (command.size() != 4)
          std::cout<<"\tUsage: \n\t\tadd node id type"<<std::endl;
        else
          graph_.add_node(command[2], command[3]);
      }
      else if (command[1] == "edge")
      {

        if ((command.size() == 2) ||
           ((command.size() > 2) &&
             ((command[2] != "string") && (command[2] != "double") && (command[2] != "tf"))))
          std::cout<<"\t\tadd edge [string|double|tf] source target data"<<std::endl;
        else if (command[2] == "string")
        {

          if (command.size() >= 6)
          {
            std::string data = command[5];
            for (int i = 6; i < command.size(); i++) data = data + " " + command[i];
            graph_.add_edge(command[3], data, command[4]);
          }
          else
            std::cout<<"\t\tadd edge string source target data"<<std::endl;
        }
        else if (command[2] == "double")
        {
          if (command.size() == 6)
          {
            try
            {
              graph_.add_edge(command[3], std::stod(command[5]), command[4]);
            } catch (std::invalid_argument& e)
            {
              std::cout<<"Please, introduce a correct double number: "<<e.what()<<std::endl;
              std::cout<<"\t\tadd edge [string|double|tf] source target data"<<std::endl;
            }
          } else
            std::cout<<"\t\tadd edge double source target data"<<std::endl;
        } else if (command[2] == "tf")
        {
          if (command.size() != 11)
            std::cout<<"Please, introduce a correct coordinate in format x  y  z y  p  r: "
          << std::endl;
          else
          {
            try
            {
              double x = std::stod(command[5]);
              double y = std::stod(command[6]);
              double z = std::stod(command[7]);
              double ry = std::stod(command[8]);
              double rp = std::stod(command[9]);
              double rr = std::stod(command[10]);
              tf2::Quaternion q;
              q.setRPY(rr, rp, ry);
              tf2::Transform trans(q, tf2::Vector3(x, y, z));
              graph_.add_edge(command[3], trans, command[4]);
            } catch (std::invalid_argument& e)
            {
              std::cout<<"Please, introduce a correct coordinate between quotation marks in format \"x  y  z y p r\": "<<e.what()<<std::endl;
              std::cout<<"\t\tadd edge [string|double|tf] source target data"<<std::endl;
            } catch (bica_graph::exceptions::NodeNotFound& e)
            {
              std::cout<<"\tNode not found: " << e.what() << std::endl;
            }
          }  // if ((command[5].a ...
        } else {
          std::cout<<"\tUsage: \n\t\tadd edge [string|double|tf]..."<<std::endl;
        }  // else if (command[2] == "tf") ...
      } else
      {
        std::cout<<"\tUsage: \n\t\tadd [node|edge]..."<<std::endl;
      }
    }
  }


  void process_remove(const std::vector<std::string>& command)
  {
    if (command.size() > 1)
    {
      if (command[1] == "node")
      {
        if (command.size() != 3)
          std::cout<<"\tUsage: \n\t\tremove node id"<<std::endl;
        else
        {
          try
          {
            graph_.remove_node(command[2]);
          } catch(bica_graph::exceptions::NodeNotFound& e)
          {
            std::cout<<"\tNode not found: " << e.what() << std::endl;
          }
        }
      } else if (command[1] == "edge")
      {
        if ((command.size() == 2) ||
           ((command.size() > 2) &&
             ((command[2] != "string") && (command[2] != "double") && (command[2] != "tf"))))
          std::cout<<"\t\tremove edge [string|double|tf] source target [data]"<<std::endl;
        else if (command[2] == "string")
        {
          if (command.size() >= 6)
          {
            std::string data = command[5];
            for (int i = 6; i < command.size(); i++) data = data + " " + command[i];
            graph_.remove_edge(command[3], data, command[4]);
          }else
            std::cout<<"\t\tremove edge string source target data"<<std::endl;
        } else if (command[2] == "double")
        {
          if (command.size() == 5)
            graph_.remove_double_edge(command[3], command[4]);
          else
            std::cout<<"\t\tremove edge double source target"<<std::endl;
        }
        else if (command[2] == "tf")
        {
          if (command.size() == 5)
            graph_.remove_tf_edge(command[3], command[4]);
          else
            std::cout<<"\t\tremove edge tf source target"<<std::endl;
        } else
        {
          std::cout<<"\t\tremove edge [string|double|tf] source target [data]"<<std::endl;
        }
      } else
      {
        std::cout<<"\tUsage: \n\t\tremove [node|edge]..."<<std::endl;
      }
    }else
    {
      std::cout<<"\tUsage: \n\t\tremove [node|edge]..."<<std::endl;
    }
  }

  void process_command(const std::string& command)
  {
    std::vector<std::string> tokens = tokenize(command);

    if (tokens.empty())
      return;

    if (tokens[0] == "list")
      process_list(tokens);
    else if (tokens[0] == "add")
      process_add(tokens);
    else if (tokens[0] == "remove")
      process_remove(tokens);
    else
      std::cout << "Command not found" << std::endl;
  }

  ~GraphTerminal()
  {
    spinner_.stop();
  }
private:
  ros::AsyncSpinner spinner_;
  bica_graph::GraphClient graph_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "graph_terminal");
  ros::NodeHandle nh;

  GraphTerminal terminal;
  terminal.run_console();

  ros::waitForShutdown();

  return 0;
}
