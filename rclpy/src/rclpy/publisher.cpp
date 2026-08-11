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
#include <rcl/publisher.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rmw/serialized_message.h>

#include <memory>
#include <string>

#include "exceptions.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "utils.hpp"

namespace rclpy
{
Publisher::Publisher(
  Node & node, py::object pymsg_type, std::string topic,
  py::object pyqos_profile, py::object pyconstraints)
: node_(node),
  pymsg_type_(pymsg_type),
  type_support_(nullptr),
  is_cpython_(false)
{
  auto msg_type = static_cast<rosidl_message_type_support_t *>(
    common_get_type_support(pymsg_type, &is_cpython_));
  if (!msg_type) {
    throw py::error_already_set();
  }
  type_support_ = msg_type;

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  if (!pyqos_profile.is_none()) {
    publisher_ops.qos = pyqos_profile.cast<rmw_qos_profile_t>();
  }

  rcl_publisher_ = std::shared_ptr<rcl_publisher_t>(
    new rcl_publisher_t,
    [node](rcl_publisher_t * publisher)
    {
      // Intentionally capturing node by value so shared_ptr can be transferred to copies
      rcl_ret_t ret = rcl_publisher_fini(publisher, node.rcl_ptr());
      if (RCL_RET_OK != ret) {
        // Warning should use line number of the current stack frame
        int stack_level = 1;
        PyErr_WarnFormat(
          PyExc_RuntimeWarning, stack_level, "Failed to fini publisher: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete publisher;
    });

  *rcl_publisher_ = rcl_get_zero_initialized_publisher();

  rcl_ret_t ret;
  if (pyconstraints.is_none()) {
    ret = rcl_publisher_init(
      rcl_publisher_.get(), node_.rcl_ptr(), msg_type,
      topic.c_str(), &publisher_ops);
  } else {
    // The constraints struct is only valid for the duration of this call: the
    // rmw layer clones the constraints into owned storage during init.
    rosidl_message_type_constraints_t constraints =
      message_constraints_from_py(pyconstraints);
    ret = rcl_publisher_init_with_constraints(
      rcl_publisher_.get(), node_.rcl_ptr(), msg_type, &constraints,
      topic.c_str(), &publisher_ops);
  }
  if (RCL_RET_OK != ret) {
    if (RCL_RET_TOPIC_NAME_INVALID == ret) {
      std::string error_text{"Failed to create publisher due to invalid topic name '"};
      error_text += topic;
      error_text += "'";
      throw py::value_error(error_text);
    }
    throw RCLError("Failed to create publisher");
  }
}

void Publisher::destroy()
{
  rcl_publisher_.reset();
  node_.destroy();
}

const char *
Publisher::get_logger_name()
{
  const char * node_logger_name = rcl_node_get_logger_name(node_.rcl_ptr());
  if (!node_logger_name) {
    throw RCLError("Node logger name not set");
  }

  return node_logger_name;
}

size_t
Publisher::get_subscription_count()
{
  size_t count = 0;
  rcl_ret_t ret = rcl_publisher_get_subscription_count(rcl_publisher_.get(), &count);
  if (RCL_RET_OK != ret) {
    throw RCLError("failed to get subscription count");
  }

  return count;
}

std::string
Publisher::get_topic_name()
{
  const char * topic_name = rcl_publisher_get_topic_name(rcl_publisher_.get());
  if (!topic_name) {
    throw RCLError("failed to get topic name");
  }

  return std::string(topic_name);
}

void
Publisher::publish(py::object pymsg)
{
  if (is_cpython_) {
    // CPython-typesupport messages ARE PyObject *: pass the Python object
    // directly to rcl; the outer table (dispatched by rmw) serializes it.  No
    // convert_from_py involved.
    rcl_ret_t ret = rcl_publish(rcl_publisher_.get(), pymsg.ptr(), NULL);
    if (RCL_RET_OK != ret) {
      throw RCLError("Failed to publish");
    }
    return;
  }

  auto raw_ros_message = convert_from_py(pymsg);
  if (!raw_ros_message) {
    throw py::error_already_set();
  }

  rcl_ret_t ret = rcl_publish(rcl_publisher_.get(), raw_ros_message.get(), NULL);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to publish");
  }
}

MessageLoan
Publisher::borrow_loaned_message(py::object pyconstraints)
{
  if (!is_cpython_) {
    // Message loaning requires a CPython typesupport: the loaned sample is a
    // Python message view only for such typesupports.
    throw py::type_error(
      "loaned messages are only supported for CPython typesupport messages");
  }

  void * message = nullptr;
  rcl_ret_t ret;
  if (pyconstraints.is_none()) {
    ret = rcl_borrow_loaned_message(
      rcl_publisher_.get(), type_support_, &message);
  } else {
    // Per-loan constraints are cloned into owned storage by the rmw layer, so
    // the struct only needs to be valid for the duration of this call.
    rosidl_message_type_constraints_t constraints =
      message_constraints_from_py(pyconstraints);
    ret = rcl_borrow_loaned_message_with_constraints(
      rcl_publisher_.get(), type_support_, &constraints, &message);
  }
  if (RCL_RET_OK != ret) {
    // Forward the middleware decision (e.g. RCL_RET_UNSUPPORTED with the rmw
    // error set) instead of pre-empting it in rclpy.
    throw RCLError("Failed to borrow loaned message");
  }

  // construct_message_at returned a new reference to a Python message
  // instance whose containers view the middleware-owned storage.
  py::object msg_obj =
    py::reinterpret_steal<py::object>(static_cast<PyObject *>(message));
  if (!py::isinstance(msg_obj, pymsg_type_)) {
    rcl_return_loaned_message_from_publisher(rcl_publisher_.get(), message);
    rcl_reset_error();
    throw py::type_error(
      "loaned message is not a Python message instance of the publisher's "
      "message type");
  }
  return MessageLoan(msg_obj);
}

void
Publisher::publish_loaned_message(MessageLoan & loan)
{
  if (!is_cpython_) {
    throw py::type_error(
      "loaned messages are only supported for CPython typesupport messages");
  }
  // Fetching the message is destructive: the loan is consumed by this call.
  py::object message = loan.take_message();
  if (message.is_none()) {
    throw py::type_error("loaned message is not valid");
  }
  rcl_ret_t ret = rcl_publish_loaned_message(
    rcl_publisher_.get(), message.ptr(), NULL);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to publish loaned message");
  }
}

void
Publisher::return_loaned_message(MessageLoan & loan)
{
  if (!is_cpython_) {
    throw py::type_error(
      "loaned messages are only supported for CPython typesupport messages");
  }
  // Fetching the message is destructive: the loan is consumed by this call.
  py::object message = loan.take_message();
  if (message.is_none()) {
    throw py::type_error("loaned message is not valid");
  }
  rcl_ret_t ret = rcl_return_loaned_message_from_publisher(
    rcl_publisher_.get(), message.ptr());
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to return loaned message");
  }
}

void
Publisher::publish_raw(std::string msg)
{
  rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  serialized_msg.buffer_capacity = msg.size();
  serialized_msg.buffer_length = msg.size();
  serialized_msg.buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(msg.c_str()));

  rcl_ret_t ret = rcl_publish_serialized_message(rcl_publisher_.get(), &serialized_msg, NULL);
  if (RCL_RET_OK != ret) {
    throw RCLError("Failed to publish");
  }
}

bool
Publisher::wait_for_all_acked(rcl_duration_t pytimeout)
{
  rcl_ret_t ret = rcl_publisher_wait_for_all_acked(rcl_publisher_.get(), pytimeout.nanoseconds);
  if (RCL_RET_OK == ret) {
    return true;
  } else if (RCL_RET_TIMEOUT == ret) {
    return false;
  }
  throw RCLError("Failed to wait for all acknowledgements");
}

void
define_publisher(py::object module)
{
  py::class_<Publisher, Destroyable, std::shared_ptr<Publisher>>(module, "Publisher")
  .def(
    py::init<Node &, py::object, std::string, py::object, py::object>(),
    py::arg("node"),
    py::arg("msg_type"),
    py::arg("topic"),
    py::arg("qos_profile"),
    py::arg("constraints") = py::none())
  .def_property_readonly(
    "pointer", [](const Publisher & publisher) {
      return reinterpret_cast<size_t>(publisher.rcl_ptr());
    },
    "Get the address of the entity as an integer")
  .def(
    "get_logger_name", &Publisher::get_logger_name,
    "Get the name of the logger associated with the node of the publisher")
  .def(
    "get_subscription_count", &Publisher::get_subscription_count,
    "Count subscribers from a publisher.")
  .def(
    "get_topic_name", &Publisher::get_topic_name,
    "Retrieve the topic name from a Publisher.")
  .def(
    "publish", &Publisher::publish,
    "Publish a message")
  .def(
    "publish_raw", &Publisher::publish_raw,
    "Publish a serialized message.")
  .def(
    "borrow_loaned_message", &Publisher::borrow_loaned_message,
    py::arg("constraints") = py::none(),
    "Borrow a loaned message from the middleware.")
  .def(
    "publish_loaned_message", &Publisher::publish_loaned_message,
    "Publish a previously borrowed loaned message.")
  .def(
    "return_loaned_message", &Publisher::return_loaned_message,
    "Return a previously borrowed loaned message without publishing it.")
  .def(
    "wait_for_all_acked", &Publisher::wait_for_all_acked,
    "Wait until all published message data is acknowledged");
}
}  // namespace rclpy
