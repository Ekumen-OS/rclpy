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

#ifndef RCLPY__LOAN_HPP_
#define RCLPY__LOAN_HPP_

#include <pybind11/pybind11.h>

#include <utility>

namespace py = pybind11;

namespace rclpy
{

/// A pure, transparent holder for a loaned message.
/**
 * MessageLoan wraps a Python message instance whose memory is owned by the
 * middleware (a loan).  It does not borrow or return anything by itself: the
 * lending entities (publisher / subscription) drive the loan lifecycle.
 *
 * It is generic on the message type (the wrapped message is an opaque Python
 * object) and transparent: attribute access is forwarded to the wrapped
 * message.  The wrapped message is set on construction; it cannot be read or
 * replaced from Python, and fetching it (take_message) is a destructive
 * operation that moves it out and leaves the loan empty (invalid).
 */
class MessageLoan
{
public:
  explicit MessageLoan(py::object message)
  : message_(std::move(message)) {}

  /// Destructively fetch the wrapped message, leaving the loan invalid.
  /**
   * Moves the wrapped Python message out of the loan (which becomes empty /
   * invalid).  Returns None if the loan was already empty.
   */
  py::object
  take_message()
  {
    return std::exchange(message_, py::none());
  }

  /// True while the loan still wraps a message.
  bool
  is_valid() const
  {
    return !message_.is_none();
  }

  /// Borrowed access to the wrapped message (attribute forwarding only).
  /**
   * Not exposed to Python; used internally to forward attribute access to the
   * wrapped message without consuming the loan.
   */
  py::object
  message() const
  {
    return message_;
  }

private:
  py::object message_;
};

/// Register the MessageLoan pybind class.
void
define_message_loan(py::object module);

}  // namespace rclpy

#endif  // RCLPY__LOAN_HPP_
