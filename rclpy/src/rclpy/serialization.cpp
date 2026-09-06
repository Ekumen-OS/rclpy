// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <pybind11/pybind11.h>

#include <rcl/error_handling.h>
#include <rcl/types.h>
#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <memory>

#include "exceptions.hpp"
#include "serialization.hpp"
#include "utils.hpp"

namespace rclpy
{
SerializedMessage::SerializedMessage(rcutils_allocator_t allocator)
{
  rcl_msg = rmw_get_zero_initialized_serialized_message();
  rcutils_ret_t rcutils_ret = rmw_serialized_message_init(&rcl_msg, 0u, &allocator);
  if (RCUTILS_RET_OK != rcutils_ret) {
    throw RCUtilsError("failed to initialize serialized message");
  }
}

SerializedMessage::~SerializedMessage()
{
  rcutils_ret_t ret = rmw_serialized_message_fini(&rcl_msg);
  if (RCUTILS_RET_OK != ret) {
    RCUTILS_SAFE_FWRITE_TO_STDERR(
      "[rclpy|" RCUTILS_STRINGIFY(__FILE__) ":" RCUTILS_STRINGIFY(__LINE__) "]: "
      "failed to fini rcl_serialized_msg_t in destructor:");
    RCUTILS_SAFE_FWRITE_TO_STDERR(rcutils_get_error_string().str);
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
    rcutils_reset_error();
  }
}

py::bytes
serialize(py::object pymsg, py::object pymsg_type)
{
  // Get type support
  auto ts = static_cast<rosidl_message_type_support_t *>(
    common_get_type_support(pymsg_type));
  if (!ts) {
    throw py::error_already_set();
  }

  const rosidl_runtime_cpython::MessageTypeBridge * bridge = get_type_bridge(pymsg);
  void * ros_msg = nullptr;
  std::unique_ptr<void, destroy_ros_message_function *> owned_msg(nullptr, nullptr);
  if (bridge) {
    // Experimental message: unwrap to the C++ message pointer (no ownership
    // transfer; the Python handle retains ownership).
    ros_msg = bridge->unwrap_cpp_message(pymsg.ptr());
  } else {
    owned_msg = convert_from_py(pymsg);
    if (!owned_msg) {
      throw py::error_already_set();
    }
    ros_msg = owned_msg.get();
  }

  // Create a serialized message object
  SerializedMessage serialized_msg(rcutils_get_default_allocator());

  // Serialize
  rmw_ret_t rmw_ret = rmw_serialize(ros_msg, ts, &serialized_msg.rcl_msg);
  if (RMW_RET_OK != rmw_ret) {
    throw RMWError("Failed to serialize ROS message");
  }

  // Bundle serialized message in a bytes object
  return py::bytes(
    reinterpret_cast<const char *>(serialized_msg.rcl_msg.buffer),
    serialized_msg.rcl_msg.buffer_length);
}

py::object
deserialize(py::bytes pybuffer, py::object pymsg_type)
{
  // Get type support
  auto ts = static_cast<rosidl_message_type_support_t *>(
    common_get_type_support(pymsg_type));
  if (!ts) {
    throw py::error_already_set();
  }

  // Create a serialized message object
  rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  // Just copy pointer to avoid extra allocation and copy
  char * serialized_buffer;
  Py_ssize_t length;
  if (PYBIND11_BYTES_AS_STRING_AND_SIZE(pybuffer.ptr(), &serialized_buffer, &length)) {
    throw py::error_already_set();
  }
  if (length < 0) {
    throw py::error_already_set();
  }
  serialized_msg.buffer_capacity = length;
  serialized_msg.buffer_length = length;
  serialized_msg.buffer = reinterpret_cast<uint8_t *>(serialized_buffer);

  const rosidl_runtime_cpython::MessageTypeBridge * bridge = get_type_bridge(pymsg_type);
  py::object deserialized_msg;
  void * ros_msg = nullptr;
  std::unique_ptr<void, destroy_ros_message_function *> owned_msg(nullptr, nullptr);
  if (bridge) {
    // Experimental message: create an owning Python handle and deserialize
    // into its C++ message.
    deserialized_msg = pymsg_type();
    ros_msg = bridge->unwrap_cpp_message(deserialized_msg.ptr());
  } else {
    owned_msg = create_from_py(pymsg_type);
    if (!owned_msg) {
      throw py::error_already_set();
    }
    ros_msg = owned_msg.get();
  }

  // Deserialize
  rmw_ret_t rmw_ret = rmw_deserialize(&serialized_msg, ts, ros_msg);

  if (RMW_RET_OK != rmw_ret) {
    throw RMWError("failed to deserialize ROS message");
  }

  if (bridge) {
    return deserialized_msg;
  }
  return convert_to_py(owned_msg.get(), pymsg_type);
}
}  // namespace rclpy
