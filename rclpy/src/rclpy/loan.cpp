// Copyright 2026 Ekumen, Inc.
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

#include "loan.hpp"

#include <string>
#include <utility>

namespace rclpy
{

void
define_message_loan(py::object module)
{
  py::class_<MessageLoan>(module, "MessageLoan")
  .def(py::init<py::object>(), py::arg("message"))
  .def_property_readonly(
    "valid", &MessageLoan::is_valid,
    "True while the loan still wraps a message.")
  .def(
    "__getattr__",
    [](MessageLoan & loan, const std::string & name) -> py::object {
      if (!loan.is_valid()) {
        throw py::cast_error(
          "MessageLoan has already been published or returned and can no "
          "longer be used");
      }
      return loan.message().attr(name.c_str());
    },
    "Forward attribute access to the wrapped message.")
  .def(
    "__setattr__",
    [](MessageLoan & loan, const std::string & name, py::object value) {
      if (!loan.is_valid()) {
        throw py::cast_error(
          "MessageLoan has already been published or returned and can no "
          "longer be used");
      }
      loan.message().attr(name.c_str()) = std::move(value);
    })
  .def(
    "__repr__",
    [](const MessageLoan & loan) -> std::string {
      if (!loan.is_valid()) {
        return std::string("MessageLoan(invalid)");
      }
      return std::string("MessageLoan(") +
             py::str(py::repr(loan.message())).cast<std::string>() + ")";
    });
}

}  // namespace rclpy
