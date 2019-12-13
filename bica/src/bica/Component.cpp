/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
/*
 * Component.cpp
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#include "bica/Component.hpp"

#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "bica_msgs/srv/activate_component.hpp"
#include "bica_msgs/srv/deactivate_component.hpp"

#include "rcutils/logging_macros.h"

namespace bica
{

using namespace std::chrono_literals;
using namespace lifecycle_msgs::msg;

Component::Component(const std::string & id, float rate)
: rclcpp_lifecycle::LifecycleNode(id), rate_(rate)
{
  using namespace std::placeholders;

  activation_pub_ = create_publisher<std_msgs::msg::String>("/bica_activations",
      rclcpp::QoS(1).reliable());
  activation_sub_ = create_subscription<std_msgs::msg::String>("/bica_activations",
    rclcpp::QoS(10).reliable(),
    std::bind(&Component::activations_callback, this, _1));

  activation_service_ = create_service<bica_msgs::srv::ActivateComponent>(
    "~/activate",
    std::bind(&Component::activate_callback, this, _1, _2,_3)
  );
  
  deactivation_service_ = create_service<bica_msgs::srv::DeactivateComponent>(
    "~/deactivate",
    std::bind(&Component::deactivate_callback, this, _1, _2,_3)
  );
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
Component::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_configure start");
  activation_pub_->on_activate();

  notifyStart();
  
  RCLCPP_INFO(get_logger(), "on_configure end");
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Component::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_activate start");
  activateDependencies();
  RCLCPP_INFO(get_logger(), "on_activate end");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Component::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_deactivate start");
  deActivateDependencies();
  RCLCPP_INFO(get_logger(), "on_deactivate end");
 
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Component::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_cleanup start");
  deActivateDependencies();
  RCLCPP_INFO(get_logger(), "on_cleanup end");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Component::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_shutdown start");
  deActivateDependencies();
  RCLCPP_INFO(get_logger(), "on_shutdown end");

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
Component::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_error start");
  deActivateDependencies();
  RCLCPP_INFO(get_logger(), "on_error end");

  return CallbackReturnT::SUCCESS;
}

void
Component::addDependency(const std::string& dep)
{
  dependencies_.insert(dep);
}

void
Component::removeDependency(const std::string& dep)
{
 dependencies_.erase(dep);
}

bool
Component::isDependency(const std::string& dep)
{
  return dependencies_.find(dep) != dependencies_.end();
}

void
Component::notifyStart()
{
  RCLCPP_INFO(get_logger(), "notifyStart start");
  std_msgs::msg::String msg;
  msg.data = get_name();

  activation_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "notifyStart end");
}

void
Component::activate_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<bica_msgs::srv::ActivateComponent::Request> request,
  const std::shared_ptr<bica_msgs::srv::ActivateComponent::Response> response)
{
  RCLCPP_INFO(get_logger(), "activate_callback start (in state %s)", this->get_current_state().label().c_str());
  if ((this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) && 
    (this->get_current_state().id() != State::TRANSITION_STATE_ACTIVATING)) {
    RCLCPP_INFO(get_logger(), "notifyStart triggering");
    
    this->trigger_transition(Transition::TRANSITION_ACTIVATE);
    activators_.insert(request->activator);
  }
  RCLCPP_INFO(get_logger(), "activate_callback end");
}

void
Component::removeActivator(const std::string & activator)
{
  if (activators_.erase(activator) == 0) {
    RCLCPP_WARN(get_logger(), "Request for deactivation from a non valid activator [%s]",
    activator.c_str());
  }

  if (activators_.empty()) {  
    this->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  }
}

void
Component::deactivate_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Request> request,
  const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Response> response)
{
  RCLCPP_INFO(get_logger(), "deactivate_callback start (in state %s)", this->get_current_state().label().c_str());
  if ((this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)) {

    RCLCPP_INFO(get_logger(), "deactivate_callback triggering");

    removeActivator(request->deactivator);
  }
  RCLCPP_INFO(get_logger(), "deactivate_callback end");
}

void
Component::activateDependency(const std::string & dep)
{
  RCLCPP_INFO(get_logger(), "activateDependency start --> %s", dep.c_str());
  auto aux = std::make_shared<rclcpp::Node>(get_name() + std::string("_actaux"));
  auto client = aux->create_client<bica_msgs::srv::ActivateComponent>("/" + dep + "/activate");

  if (!client->wait_for_service(100ms)) {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.",
        client->get_service_name());
    return;
  }

  auto request = std::make_shared<bica_msgs::srv::ActivateComponent::Request>();
  request->activator = get_name();

  auto future_result = client->async_send_request(request);
  

  if (rclcpp::spin_until_future_complete(aux->get_node_base_interface(), future_result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "Component [%s] failed to activate", dep.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "activateDependency end");
}

void
Component::deactivateDependency(const std::string & dep)
{
  RCLCPP_INFO(get_logger(), "deactivateDependency start --> %s", dep.c_str());

  auto aux = std::make_shared<rclcpp::Node>(get_name() + std::string("_actaux"));
  auto client = aux->create_client<bica_msgs::srv::DeactivateComponent>("/" + dep + "/deactivate");

  if (!client->wait_for_service(100ms)) {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.",
        client->get_service_name());
    return;
  }

  auto request = std::make_shared<bica_msgs::srv::DeactivateComponent::Request>();
  request->deactivator = get_name();

  auto future_result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(aux->get_node_base_interface(), future_result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "Component [%s] failed to activate", dep.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "deactivateDependency end");
}

void
Component::activateDependencies()
{
  for (const auto & dep : dependencies_) {
    activateDependency(dep);
  }
}

void
Component::deActivateDependencies()
{
  for (const auto & dep : dependencies_) {
   deactivateDependency(dep);
  }
}

void
Component::activations_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "activations_callback start <-- %s", msg->data.c_str());
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE && isDependency(msg->data)) {
    activateDependency(msg->data);
  }
  RCLCPP_INFO(get_logger(), "activations_callback end");
}

bool
Component::ok()
{
  auto nodes = this->get_node_graph_interface()->get_node_names();
  auto check_activators = activators_;
  for (auto const & activator : check_activators) {
    if (std::find(nodes.begin(), nodes.end(), "/" + activator) == nodes.end())
    {
      RCLCPP_INFO(get_logger(), "Activator %s is not longer present, removing from activators");
      removeActivator(activator);
    }
  }

  return rclcpp::ok();
}

void
Component::execute()
{
 while (this->ok()) {
     RCLCPP_INFO(get_logger(), "state: %s", this->get_current_state().label().c_str());
     if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_INFO(get_logger(), "step");
    } 

    rclcpp::spin_some(this->get_node_base_interface());
    rate_.sleep();
  }
}


} /* namespace bica */
