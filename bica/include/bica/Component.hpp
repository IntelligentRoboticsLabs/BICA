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

#ifndef BICA__COMPONENT_HPP_
#define BICA__COMPONENT_HPP_

#include <string>
#include <set>
#include <memory>
#include <vector>

#include "bica/Utils.hpp"

#include "std_msgs/msg/string.hpp"
#include "bica_msgs/srv/activate_component.hpp"
#include "bica_msgs/srv/deactivate_component.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bica
{


struct ActivationFuture
{
  using ActivateComponent_Response =
    std::shared_ptr<bica_msgs::srv::ActivateComponent_Response_<std::allocator<void>>>;

  std::shared_future<ActivateComponent_Response> future;
  std::string component;
};

struct DeactivationFuture
{
  using DeactivateComponent_Response =
    std::shared_ptr<bica_msgs::srv::DeactivateComponent_Response_<std::allocator<void>>>;

  std::shared_future<DeactivateComponent_Response> future;
  std::string component;
};

class Component : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// Creates ac BICA component
  /**
   * \param[in] id The name of this component
   */
  explicit Component(const std::string & id, float rate = 1.0);
  virtual ~Component() {}

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// This method contains the control loop. It is blocking
  virtual void execute();

  /// This method contains one step of the control loop
  /**
   * \param[in] spin If true, it calls to rclcpp::spin_once and sleep to achive the
   *            execution rate. If False, user should call control freq and spin it.
   */
  virtual void execute_once(bool spin = true);

  /// Configures domain by creating a DomainExpert object
  /**
   * \param[in] state LifeCycle Node's state
   * \return Success or Failure
   */
  virtual CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

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

  /// It contains the functionality of the process
  virtual void step() {}

  /// Returns the rate (as rclcpp:Rate) at which this component was configured
  /**
   * \return Configured rate
   */
  rclcpp::Rate & get_rate() {return rate_;}

  /// Returns the rate (as float) at which this component was configured
  /**
   * \return Configured rate
   */
  float get_rate_as_freq() {return rate_freq_;}

  void addDependency(const std::string & dep);
  void removeDependency(const std::string & dep);

protected:
  virtual void on_activate() {}
  virtual void on_deactivate() {}

private:
  rclcpp::Rate rate_;
  float rate_freq_;

  std::set<std::string> dependencies_;
  std::set<std::string> activators_;

  std::vector<ActivationFuture> pending_act_futures_;
  std::vector<DeactivationFuture> pending_deact_futures_;

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
