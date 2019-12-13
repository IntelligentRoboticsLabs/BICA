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
 * Component.h
 *
 *  Created on: 11/05/2016
 *      Author: paco
 */

#ifndef BICA__COMPONENT_HPP_
#define BICA__COMPONENT_HPP_

#include <string>
#include <set>

#include "bica/Utils.hpp"

#include "std_msgs/msg/string.hpp"
#include "bica_msgs/srv/activate_component.hpp"
#include "bica_msgs/srv/deactivate_component.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bica
{

using namespace std::chrono_literals;

class Component : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// Creates ac BICA component
  /**
   * \param[in] id The name of this component
   */
  Component(const std::string & id, float rate = 1.0);
  virtual ~Component() {}

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// This method has to be called iterativelly 
  virtual void execute();

  /// Configures domain by creating a DomainExpert object
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /// Activates the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /// Deactivates the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /// Cleans up the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /// Shuts down the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /// Manages the error in the node
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /// Check if rclcpp is ok, and periodic tasks
  /**
   * \return rclcpp::ok();
   */
  bool ok();

protected:
  void addDependency(const std::string & dep);
  void removeDependency(const std::string & dep);

private:
  rclcpp::Rate rate_;

  std::set<std::string> dependencies_;
  std::set<std::string> activators_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr activation_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr activation_sub_;
  rclcpp::Service<bica_msgs::srv::ActivateComponent>::SharedPtr activation_service_;
  rclcpp::Service<bica_msgs::srv::DeactivateComponent>::SharedPtr deactivation_service_;

  void notifyStart();
  bool isDependency(const std::string & dep);
  void activateDependency(const std::string & dep);
  void deactivateDependency(const std::string & dep);
  void removeActivator(const std::string & activator);

  void activateDependencies();
  void deActivateDependencies();

  void activations_callback(const std_msgs::msg::String::SharedPtr msg);
  void activate_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<bica_msgs::srv::ActivateComponent::Request> request,
  const std::shared_ptr<bica_msgs::srv::ActivateComponent::Response> response);
  void deactivate_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Request> request,
  const std::shared_ptr<bica_msgs::srv::DeactivateComponent::Response> response);
};

}  // namespace bica

#endif  // BICA__COMPONENT_HPP_
