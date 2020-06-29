// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "bica/Component.hpp"
#include "rclcpp/rclcpp.hpp"


class CompC : public bica::Component
{
public:
  CompC()
  : bica::Component("C", 10)
  {
  }

  void step()
  {
    RCLCPP_INFO(get_logger(), "CompC::step()");
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto component = std::make_shared<CompC>();

  while (rclcpp::ok()) {
    component->execute_once();
  }

  rclcpp::shutdown();

  return 0;
}
