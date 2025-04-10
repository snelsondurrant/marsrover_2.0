// Modified from the default Nav2 Spin behavior - Nelson Durrant, Apr 2025
// https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/include/nav2_behaviors/plugins/spin.hpp
// https://docs.nav2.org/plugin_tutorials/docs/writing_new_behavior_plugin.html

// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIORS__PLUGINS__CUSTOM_SPIN_HPP_
#define NAV2_BEHAVIORS__PLUGINS__CUSTOM_SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace rover_behaviors
{
using SpinAction = nav2_msgs::action::Spin;

/**
 * @class nav2_behaviors::CustomSpin
 * @brief An action server behavior for spinning in
 */
class CustomSpin : public nav2_behaviors::TimedBehavior<SpinAction>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::CustomSpin
   */
  CustomSpin();
  ~CustomSpin();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
   nav2_behaviors::Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
   nav2_behaviors::Status onCycleUpdate() override;

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  SpinAction::Feedback::SharedPtr feedback_;

  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__CUSTOM_SPIN_HPP_