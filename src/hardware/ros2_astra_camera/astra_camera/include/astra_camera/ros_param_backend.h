/**************************************************************************/
/* */
/* Copyright (c) 2013-2023 Orbbec 3D Technology, Inc                      */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/* */
/**************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <functional>

namespace astra_camera {

class ParametersBackend {
 public:
  /** * @brief New Callback type definition compatible with ROS 2 Jazzy
   */
  using OnParametersSetCallbackType = std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)>;

  explicit ParametersBackend(rclcpp::Node* node);
  ~ParametersBackend();

  void addOnSetParametersCallback(OnParametersSetCallbackType callback);

 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  
  /** * @brief Explicit handle type for the parameter callback
   */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ros_callback_;
};

}  // namespace astra_camera