# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Tests for constraints, experimental messages and loaned messages in rclpy.

These tests exercise the CPython typesupport integration: constrained
publisher/subscription creation, experimental message publish/subscribe, and
the publisher loaned-message API (borrow / mutate / publish / return).  They
require the XCDR serialization backend, enabled via the
``RMW_FASTRTPS_SERIALIZATION_BACKEND`` environment variable before rclpy init.
"""

import os
import time

os.environ['RMW_FASTRTPS_SERIALIZATION_BACKEND'] = 'xcdr'

import pytest  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from rosidl_runtime_cpython.constraints import StringConstraint  # noqa: E402

from test_msgs.msg import Strings as StdStrings  # noqa: E402
from test_msgs.msg.experimental import Strings  # noqa: E402
from test_msgs.msg.experimental import WStrings  # noqa: E402

NODE_NAME = 'test_loaned_node'
TOPIC = 'test_loaned_topic'

# Constrained endpoints register per (topic, bounds) in the shared participant,
# so every test needs its own topic name.
_topic_counter = 0


def _unique_topic():
    global _topic_counter
    _topic_counter += 1
    return f'{TOPIC}_{_topic_counter}'


@pytest.fixture(scope='session', autouse=True)
def setup_ros():
    rclpy.init()


@pytest.fixture
def node():
    n = Node(NODE_NAME)
    yield n
    n.destroy_node()


def _spin_until(node, predicate, timeout=4.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _strings_constraints(size=100):
    constraints = Strings.Constraints()
    constraints.string_value = StringConstraint(size=size)
    return constraints


def test_constrained_publisher_and_subscription_creation(node):
    pub = node.create_publisher(Strings, _unique_topic(), 10, constraints=_strings_constraints())
    sub = node.create_subscription(Strings, _unique_topic(), lambda msg: None, 10)
    assert pub is not None
    assert sub is not None


def test_invalid_constraints_type_raises(node):
    with pytest.raises(TypeError):
        node.create_publisher(Strings, _unique_topic(), 10, constraints='not constraints')


def test_constraints_rejected_for_standard_messages(node):
    # Standard message classes have no Constraints inner class; any constraints
    # argument is rejected with a clear TypeError.
    with pytest.raises(TypeError):
        node.create_publisher(StdStrings, _unique_topic(), 10, constraints=object())


def test_experimental_publish_subscribe_roundtrip(node):
    received = []
    topic = _unique_topic()
    node.create_subscription(
        Strings, topic, lambda msg: received.append(str(msg.string_value)), 10)
    pub = node.create_publisher(Strings, topic, 10)
    time.sleep(0.5)
    msg = Strings()
    msg.string_value = 'hello experimental'
    pub.publish(msg)
    assert _spin_until(node, lambda: bool(received))
    assert ['hello experimental'] == received


def test_borrow_mutate_publish_roundtrip(node):
    received = []
    topic = _unique_topic()
    pub = node.create_publisher(Strings, topic, 10, constraints=_strings_constraints())
    node.create_subscription(
        Strings, topic, lambda msg: received.append(str(msg.string_value)), 10)
    time.sleep(0.5)
    with pub.borrow_loaned_message() as loan:
        loan.string_value = 'loaned hello'
        pub.publish_loaned_message(loan)
    assert _spin_until(node, lambda: bool(received))
    assert ['loaned hello'] == received


def test_borrow_at_bound_release_fast_path(node):
    # A message whose variable-length field is exactly at the constraint
    # bound takes the release (no-rewrite) fast path.
    received = []
    topic = _unique_topic()
    pub = node.create_publisher(Strings, topic, 10, constraints=_strings_constraints(size=100))
    node.create_subscription(
        Strings, topic, lambda msg: received.append(str(msg.string_value)), 10)
    time.sleep(0.5)
    with pub.borrow_loaned_message() as loan:
        loan.string_value = 'x' * 100
        pub.publish_loaned_message(loan)
    assert _spin_until(node, lambda: bool(received))
    assert ['x' * 100] == received


def test_borrow_auto_return_on_context_exit(node):
    pub = node.create_publisher(Strings, _unique_topic(), 10, constraints=_strings_constraints())
    with pub.borrow_loaned_message() as loan:
        loan.string_value = 'returned'
    # After context exit the loan was returned; accessing it raises.
    with pytest.raises(RuntimeError):
        loan.string_value


def test_post_publish_access_raises(node):
    pub = node.create_publisher(Strings, _unique_topic(), 10, constraints=_strings_constraints())
    with pub.borrow_loaned_message() as loan:
        loan.string_value = 'published'
        pub.publish_loaned_message(loan)
    with pytest.raises(RuntimeError):
        loan.string_value


def test_standard_message_loaning_disallowed(node):
    # Message loaning requires a CPython typesupport; standard messages use
    # the C typesupport, so the loaning methods raise early.
    pub = node.create_publisher(StdStrings, _unique_topic(), 10)
    with pytest.raises(TypeError):
        with pub.borrow_loaned_message():
            pass
    with pytest.raises(TypeError):
        pub.publish_loaned_message(object())


def test_constraint_violation_on_borrowed_message(node):
    # The borrowed String container is bound-sized; assigning a longer
    # value raises instead of overflowing the loan.
    pub = node.create_publisher(
        Strings, _unique_topic(), 10, constraints=_strings_constraints(size=100))
    with pub.borrow_loaned_message() as loan:
        with pytest.raises(Exception):
            loan.string_value = 'x' * 101


def test_loaned_wstring_array_roundtrip(node):
    constraints = WStrings.Constraints()
    constraints.wstring_value = StringConstraint(size=100)
    constraints.wstring_value_default1 = StringConstraint(size=100)
    constraints.wstring_value_default2 = StringConstraint(size=100)
    constraints.wstring_value_default3 = StringConstraint(size=100)
    constraints.array_of_wstrings = StringConstraint(size=50)
    received = []
    topic = _unique_topic()
    pub = node.create_publisher(WStrings, topic, 10, constraints=constraints)

    def _w_cb(msg):
        received.append(
            (str(msg.wstring_value), [str(x) for x in msg.array_of_wstrings]))

    node.create_subscription(WStrings, topic, _w_cb, 10)
    time.sleep(0.5)
    with pub.borrow_loaned_message() as loan:
        loan.wstring_value = 'wstring hello'
        loan.array_of_wstrings = ['a', 'b', 'c']
        pub.publish_loaned_message(loan)
    assert _spin_until(node, lambda: bool(received))
    assert [('wstring hello', ['a', 'b', 'c'])] == received
