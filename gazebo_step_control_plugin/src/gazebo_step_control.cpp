// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <gz/transport/Node.hh>
#include <gz/sim/Util.hh>
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Name.hh"
#include <gz/sim/Events.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/empty.hpp>
#include "gazebo_step_control_interface/srv/step_control.hpp"
#include <gazebo_step_control_interface/srv/copy_param.hpp>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace gz;
using namespace sim;
using namespace systems;

class GazeboStepControl : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  /// Constructor
  GazeboStepControl();
  ~GazeboStepControl();

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  /// Step control after every world update done.
  void UpdateEnd(void);

  /// Callback from ROS service to enable/disable step control.
  /// \param[in] req SetBool request
  /// \param[out] res SetBool response
  void OnUpdateControl(
      std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res);

  /// To enable/disable step control.
  /// \param[in] control_status
  void UpdateControl(bool step_control_status);

  /// Callback from ROS service for step control.
  /// \param[in] req StepControl request
  /// \param[out] res StepControl response
  void OnStepControl(
      gazebo_step_control_interface::srv::StepControl::Request::SharedPtr req,
      gazebo_step_control_interface::srv::StepControl::Response::SharedPtr res);

  void SetPaused(bool pause);

  /// Gazebo-ROS node
  rclcpp::Node::SharedPtr ros_node_;

  /// Publish step complete event
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr step_complete_pub_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enablecontrol_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr namespace_reset_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<gazebo_step_control_interface::srv::StepControl>::SharedPtr stepcontrol_service_;

  /// Holds step control status
  bool step_control_status_;

  /// Number of steps to execute
  int64_t steps_to_execute_;

  /// If the service call to be blocked untill all steps executed
  bool step_blocking_call_;

  int m_fd;
  char *m_map;
  int i;

  /// World name
  std::string worldName;

  /// World entity
  gz::sim::Entity world_;

  // Transport node
  gz::transport::Node node;

  bool paused_;
};

GazeboStepControl::GazeboStepControl()
    : step_control_status_(false), steps_to_execute_(0), step_blocking_call_(false), paused_(false)
{
}

void GazeboStepControl::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &_eventMgr)
{
  world_ = _ecm.EntityByComponents(components::World());
  this->worldName = _ecm.Component<gz::sim::components::Name>(world_)->Data();

  enablecontrol_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "step_control_enable",
      std::bind(
          &GazeboStepControl::OnUpdateControl, this,
          std::placeholders::_1, std::placeholders::_2));

  stepcontrol_service_ = ros_node_->create_service<gazebo_step_control_interface::srv::StepControl>(
      "step",
      std::bind(
          &GazeboStepControl::OnStepControl, this,
          std::placeholders::_1, std::placeholders::_2));

  // Offer transient local durability on the clock topic so that if publishing is infrequent (e.g.
  // the simulation is paused), late subscribers can receive the previously published message(s).
  step_complete_pub_ = ros_node_->create_publisher<std_msgs::msg::Empty>(
      "/step_completed",
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

  // Step control flag in plugin sdf
  auto control_status_sdf = _sdf->Get<bool>("enable_control", false).first;

  // Step control parameter
  auto enable_control_param = ros_node_->declare_parameter(
      "enable_control",
      rclcpp::ParameterValue(control_status_sdf));

  UpdateControl(enable_control_param.get<bool>());
}

void GazeboStepControl::PreUpdate(const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm)
{
  if (step_control_status_)
  {
    UpdateEnd();
  }
  else if(paused_)
  {
    SetPaused(false);
  }
}

void GazeboStepControl::UpdateEnd(void)
{
  if (step_control_status_ == true)
  {
    steps_to_execute_--;
    if (steps_to_execute_ <= 0)
    {
      // world_->SetPaused(true);
      SetPaused(true);

      // publish completion topic only for non blocking service call
      if (!step_blocking_call_)
        step_complete_pub_->publish(std_msgs::msg::Empty());
    }
  }
  m_map[0] = i++;
}

void GazeboStepControl::OnUpdateControl(
    std_srvs::srv::SetBool::Request::SharedPtr _req,
    std_srvs::srv::SetBool::Response::SharedPtr _res)
{
  // UpdateControl(_req->data);
  step_control_status_ = _req->data;
  _res->success = true;
}

void GazeboStepControl::OnStepControl(
    gazebo_step_control_interface::srv::StepControl::Request::SharedPtr _req,
    gazebo_step_control_interface::srv::StepControl::Response::SharedPtr _res)
{
  steps_to_execute_ = _req->steps;
  step_blocking_call_ = _req->block;
  // Unpause physics on each step service call
  if (steps_to_execute_ > 0)
  {
    SetPaused(false);
    if (step_blocking_call_)
    {
      while (steps_to_execute_ > 0)
        usleep(1000);
    }
  }
  _res->success = true;
}

void GazeboStepControl::SetPaused(bool pause)
{
  gz::msgs::WorldControl req;
  req.set_pause(pause);

  gz::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;

  std::string service = "/world/" + worldName + "/control";

  bool executed = node.Request(service, req, timeout, rep, result);

  if (executed)
  {
    if (result){
      std::cout << "Response: [" << rep.data() << "]" << std::endl;
      paused_ = pause;
    }
    else
      std::cout << "Service call failed" << std::endl;
  }
  else
    std::cerr << "Service call timed out" << std::endl;
}

GZ_ADD_PLUGIN(GazeboStepControl,
              gz::sim::System,
              GazeboStepControl::ISystemConfigure,
              GazeboStepControl::ISystemPreUpdate)