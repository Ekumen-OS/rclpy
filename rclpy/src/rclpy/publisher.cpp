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
#include <rcutils/logging_macros.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcpputils/scope_exit.hpp>
#include "exceptions.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "utils.hpp"

namespace rclpy
{
PublisherLoan::PublisherLoan(
  std::shared_ptr<rcl_publisher_t> publisher, void * message,
  std::shared_ptr<rosidl_runtime_cpython::BorrowedViewState> state, py::object view,
  const rosidl_runtime_cpython::MessageTypeBridge * bridge,
  std::shared_ptr<void> owned, std::shared_ptr<void> constraints)
: publisher_(std::move(publisher)), message_(message), state_(std::move(state)),
  view_(std::move(view)), bridge_(bridge), owned_(std::move(owned)),
  constraints_(std::move(constraints))
{}

PublisherLoan::~PublisherLoan()
{
  if (message_) {
    state_->invalidate();
    if (owned_) {
      return;
    }
    auto ret = rcl_return_loaned_message_from_publisher(publisher_.get(), message_);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclpy", "Failed to return region publisher loan: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
}

py::object PublisherLoan::view() const
{
  if (!message_) {
    throw std::logic_error("The publisher loan is no longer valid");
  }
  return view_;
}

bool PublisherLoan::valid() const {return message_ != nullptr;}

void PublisherLoan::return_loan()
{
  if (!message_) {
    throw std::logic_error("The publisher loan is no longer valid");
  }
  state_->invalidate();
  if (owned_) {
    owned_.reset();
    message_ = nullptr;
    return;
  }
  auto ret = rcl_return_loaned_message_from_publisher(publisher_.get(), message_);
  if (ret != RCL_RET_OK) {
    throw RCLError("Failed to return region publisher loan");
  }
  message_ = nullptr;
}

void PublisherLoan::publish(
  const std::shared_ptr<rcl_publisher_t> & publisher, const std::shared_ptr<void> & baseline)
{
  if (!message_ || publisher_.get() != publisher.get()) {
    throw py::value_error("Region loan does not belong to this publisher");
  }
  if (owned_) {
    if (!bridge_->view_compatible(baseline.get(), message_) ||
      !bridge_->view_compatible(constraints_.get(), message_))
    {
      state_->invalidate();
      owned_.reset();
      message_ = nullptr;
      throw py::value_error("Region message exceeds loan constraints");
    }
    auto plain = bridge_->view_to_plain(message_);
    auto ret = rcl_publish(publisher_.get(), plain.get(), nullptr);
    if (ret != RCL_RET_PUBLISHER_INVALID) {
      state_->invalidate();
      owned_.reset();
      message_ = nullptr;
    }
    if (ret != RCL_RET_OK) {
      throw RCLError("Failed to publish region copy");
    }
    return;
  }
  auto ret = rcl_publish_loaned_message(publisher_.get(), message_, nullptr);
  if (ret != RCL_RET_PUBLISHER_INVALID) {
    state_->invalidate();
    message_ = nullptr;
  }
  if (ret != RCL_RET_OK) {
    throw RCLError("Failed to publish region loan");
  }
}

Publisher::Publisher(
  Node & node, py::object pymsg_type, std::string topic,
  py::object pyqos_profile, py::object constraints)
: node_(node), bridge_(get_view_bridge(pymsg_type))
{
  if (!bridge_ && !constraints.is_none()) {
    throw py::type_error("Constraints require a region message type");
  }
  if (bridge_) {
    native_ = region_loan_available(RMW_REGION_LOAN_PUBLISHER) &&
      region_loan_type_available(RMW_REGION_LOAN_PUBLISHER, *bridge_, constraints);
  }
  type_support_ = bridge_ ? (native_ ? bridge_->type_support() : bridge_->plain_type_support()) :
    static_cast<rosidl_message_type_support_t *>(common_get_type_support(pymsg_type));
  if (!type_support_) {
    if (!PyErr_Occurred()) {
      throw py::value_error("Message type support is null");
    }
    throw py::error_already_set();
  }

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
        warn_fini_failure("publisher");
      }
      delete publisher;
    });

  *rcl_publisher_ = rcl_get_zero_initialized_publisher();

  rcl_ret_t ret;
  if (bridge_ && native_) {
    ViewMessageContext context(constraints, *bridge_);
    ret = rcl_publisher_init_with_context(
      rcl_publisher_.get(), node_.rcl_ptr(), type_support_, context.get(),
      topic.c_str(), &publisher_ops);
  } else {
    if (bridge_) {
      ViewMessageContext context(std::move(constraints), *bridge_);
      constraints_ = copy_region_constraints(*bridge_, context.get());
      publisher_ops.disable_loaned_message = true;
    }
    ret = rcl_publisher_init(
      rcl_publisher_.get(), node_.rcl_ptr(), type_support_,
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
  if (bridge_ && native_ && !rcl_publisher_can_loan_messages(rcl_publisher_.get())) {
    ViewMessageContext context(std::move(constraints), *bridge_);
    constraints_ = copy_region_constraints(*bridge_, context.get());
    ret = rcl_publisher_fini(rcl_publisher_.get(), node_.rcl_ptr());
    if (ret != RCL_RET_OK) {
      throw RCLError("Failed to finalize non-loaning publisher");
    }
    *rcl_publisher_ = rcl_get_zero_initialized_publisher();
    type_support_ = bridge_->plain_type_support();
    publisher_ops.disable_loaned_message = true;
    ret = rcl_publisher_init(
      rcl_publisher_.get(), node_.rcl_ptr(), type_support_, topic.c_str(), &publisher_ops);
    if (ret != RCL_RET_OK) {
      throw RCLError("Failed to create copy publisher");
    }
    native_ = false;
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
  if (bridge_) {
    throw py::type_error("Region publishers require a loaned message");
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

std::shared_ptr<PublisherLoan>
Publisher::borrow_loaned_message(py::object constraints)
{
  if (!bridge_) {
    throw py::type_error("Loaned region messages require a region publisher");
  }
  auto state = std::make_shared<rosidl_runtime_cpython::BorrowedViewState>();
  auto publisher = rcl_publisher_;
  if (!native_) {
    ViewMessageContext context(std::move(constraints), *bridge_);
    auto limits = copy_region_constraints(*bridge_, context.get(), constraints_);
    auto owned = bridge_->create_view();
    auto * message = owned.get();
    auto * wrapped = bridge_->wrap_loan(message, state, false);
    if (!wrapped) {
      throw py::error_already_set();
    }
    auto view = py::reinterpret_steal<py::object>(wrapped);
    return std::make_shared<PublisherLoan>(
      publisher, message, state, std::move(view), bridge_, std::move(owned), std::move(limits));
  }
  void * message = nullptr;
  rcl_ret_t ret;
  if (constraints.is_none()) {
    ret = rcl_borrow_loaned_message_with_context(
      publisher.get(), type_support_, nullptr, &message);
  } else {
    ViewMessageContext context(std::move(constraints), *bridge_);
    ret = rcl_borrow_loaned_message_with_context(
      publisher.get(), type_support_, context.get(), &message);
  }
  if (ret != RCL_RET_OK) {
    throw RCLError("Failed to borrow region publisher loan");
  }
  if (!message) {
    throw std::runtime_error("Middleware returned a null region loan");
  }
  auto cleanup = rcpputils::make_scope_exit([&]() {
        if (message) {
          state->invalidate();
          auto return_ret = rcl_return_loaned_message_from_publisher(publisher.get(), message);
          if (return_ret != RCL_RET_OK) {
            RCUTILS_LOG_ERROR_NAMED(
            "rclpy", "Failed to return region publisher loan: %s", rcl_get_error_string().str);
            rcl_reset_error();
          }
        }
    });
  PyObject * wrapped = bridge_->wrap_loan(message, state, false);
  if (!wrapped) {
    throw py::error_already_set();
  }
  auto view = py::reinterpret_steal<py::object>(wrapped);
  auto loan = std::make_shared<PublisherLoan>(publisher, message, state, std::move(view));
  message = nullptr;
  return loan;
}

void Publisher::publish_loaned_message(PublisherLoan & loan)
{
  loan.publish(rcl_publisher_, constraints_);
}

void
Publisher::publish_raw(std::string msg)
{
  if (bridge_) {
    throw py::type_error("Region publishers require a loaned message");
  }
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
  py::class_<PublisherLoan, std::shared_ptr<PublisherLoan>>(module, "PublisherLoan")
  .def_property_readonly("view", &PublisherLoan::view)
  .def_property_readonly("valid", &PublisherLoan::valid)
  .def("return_loan", &PublisherLoan::return_loan)
  .def("__enter__", &PublisherLoan::view)
  .def(
    "__exit__", [](PublisherLoan & loan, py::object, py::object, py::object) {
      if (loan.valid()) {
        loan.return_loan();
      }
    });
  py::class_<Publisher, Destroyable, std::shared_ptr<Publisher>>(module, "Publisher")
  .def(
    py::init<Node &, py::object, std::string, py::object, py::object>(),
    py::arg("node"), py::arg("msg_type"), py::arg("topic"), py::arg("qos_profile"),
    py::arg("constraints") = py::none())
  .def_property_readonly("can_loan_messages", &Publisher::can_loan_messages)
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
    py::arg("constraints") = py::none())
  .def("publish_loaned_message", &Publisher::publish_loaned_message)
  .def(
    "wait_for_all_acked", &Publisher::wait_for_all_acked,
    "Wait until all published message data is acknowledged");
}
}  // namespace rclpy
