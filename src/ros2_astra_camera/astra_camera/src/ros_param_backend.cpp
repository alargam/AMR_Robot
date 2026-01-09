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

#include "astra_camera/ros_param_backend.h"

namespace astra_camera {

ParametersBackend::ParametersBackend(rclcpp::Node *node)
    : node_(node), logger_(node_->get_logger()) {}

ParametersBackend::~ParametersBackend() {
  /** * @brief In ROS 2 Jazzy, destroying the SharedPtr automatically clears the callback
   */
  if (ros_callback_) {
    ros_callback_.reset();
  }
}

void ParametersBackend::addOnSetParametersCallback(OnParametersSetCallbackType callback) {
  /** * @brief Register the callback using the updated ROS 2 API
   */
  ros_callback_ = node_->add_on_set_parameters_callback(std::move(callback));
}

}  // namespace astra_camera