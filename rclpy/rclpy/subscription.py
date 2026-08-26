# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from enum import Enum
import inspect
from typing import Callable
from typing import TypeVar

from rclpy.callback_groups import CallbackGroup
from rclpy.event_handler import EventHandler
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.subscription_content_filter_options import ContentFilterOptions


# Re-export exception defined in _rclpy C extension.
RCLError = _rclpy.RCLError

# For documentation only
MsgType = TypeVar('MsgType')


class SubscribedMessageLender:
    """
    A context manager that lends a message taken from the middleware.

    ``SubscribedMessageLender`` is returned by
    :meth:`.Subscription.take_loaned_message`.  Entering the context manager
    attempts a loaned take; the loan is returned to the middleware on exit.
    A failed loaned take (unsupported by the middleware, or no data available)
    does not raise: the context manager then yields None so the caller can
    fall back to a regular take.

    .. warning:: Users should not create a ``SubscribedMessageLender`` with
       this constructor; use :meth:`.Subscription.take_loaned_message` instead.
    """

    def __init__(
        self,
        subscription_impl: _rclpy.Subscription,
        msg_type: MsgType,
    ) -> None:
        self._subscription_impl = subscription_impl
        self._msg_type = msg_type
        self._loan = None

    def __enter__(self):
        # take_loaned_message returns a (MessageLoan, metadata) tuple, or None
        # when the loaned take failed.  A failed take must not raise so the
        # caller can fall back to a regular take.
        self._loan = self._subscription_impl.take_loaned_message(self._msg_type)
        return self._loan

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        # Return the loan on context exit.  This never leaks.
        if self._loan is not None and self._loan[0].valid:
            with self._subscription_impl:
                self._subscription_impl.return_loaned_message(self._loan[0])
        self._loan = None
        return False


class Subscription:

    class CallbackType(Enum):
        MessageOnly = 0
        WithMessageInfo = 1

    def __init__(
         self,
         subscription_impl: _rclpy.Subscription,
         msg_type: MsgType,
         topic: str,
         callback: Callable,
         callback_group: CallbackGroup,
         qos_profile: QoSProfile,
         raw: bool,
         event_callbacks: SubscriptionEventCallbacks,
    ) -> None:
        """
        Create a container for a ROS subscription.

        .. warning:: Users should not create a subscription with this constructor, instead they
           should call :meth:`.Node.create_subscription`.

        :param subscription_impl: :class:`Subscription` wrapping the underlying
            ``rcl_subscription_t`` object.
        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            nodes default callback group is used.
        :param qos_profile: The quality of service profile to apply to the subscription.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        self.__subscription = subscription_impl
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.callback_group = callback_group
        # True when the callback is ready to fire but has not been "taken" by an executor
        self._executor_event = False
        self.qos_profile = qos_profile
        self.raw = raw

        self.event_handlers: EventHandler = event_callbacks.create_event_handlers(
            callback_group, subscription_impl, topic)

    def get_publisher_count(self) -> int:
        """Get the number of publishers that this subscription has."""
        with self.handle:
            return self.__subscription.get_publisher_count()

    @property
    def handle(self):
        return self.__subscription

    def take_loaned_message(self) -> SubscribedMessageLender:
        """
        Attempt a loaned take from the middleware.

        Enter the returned context manager to take a
        :class:`MessageLoan`; the loan is returned to the middleware on exit.
        A failed loaned take yields None (no exception), so callers can fall
        back to a regular take.

        :return: A :class:`SubscribedMessageLender` context manager.
        """
        return SubscribedMessageLender(self.handle, self.msg_type)

    def can_loan_messages(self) -> bool:
        """
        Check whether this subscription can take loaned messages.

        Mirrors ``rcl_subscription_can_loan_messages``: false when loaned
        messages are disabled (``ROS_DISABLE_LOANED_MESSAGES``) or the
        middleware does not support them for this subscription.

        :return: ``True`` if loaned takes are supported and enabled.
        """
        with self.handle:
            return self.__subscription.can_loan_messages()

    def destroy(self):
        """
        Destroy a container for a ROS subscription.

        .. warning:: Users should not destroy a subscription with this method, instead they
           should call :meth:`.Node.destroy_subscription`.
        """
        for handler in self.event_handlers:
            handler.destroy()
        self.handle.destroy_when_not_in_use()

    @property
    def topic_name(self):
        with self.handle:
            return self.__subscription.get_topic_name()

    @property
    def callback(self):
        return self._callback

    @callback.setter
    def callback(self, value):
        self._callback = value
        self._callback_type = Subscription.CallbackType.MessageOnly
        try:
            inspect.signature(value).bind(object())
            return
        except TypeError:
            pass
        try:
            inspect.signature(value).bind(object(), object())
            self._callback_type = Subscription.CallbackType.WithMessageInfo
            return
        except TypeError:
            pass
        raise RuntimeError(
            'Subscription.__init__(): callback should be either be callable with one argument'
            '(to get only the message) or two (to get message and message info)')

    @property
    def logger_name(self) -> str:
        """Get the name of the logger associated with the node of the subscription."""
        with self.handle:
            return self.__subscription.get_logger_name()

    @property
    def is_cft_enabled(self) -> bool:
        """Check if content filtering is enabled for the subscription."""
        with self.handle:
            return self.__subscription.is_cft_enabled()

    def set_content_filter(self, filter_expression: str, expression_parameters: list[str]) -> None:
        """
        Set the filter expression and expression parameters for the subscription.

        :param filter_expression: The filter expression to set.
        :param expression_parameters: The expression parameters to set.
        :raises: RCLError if internal error occurred when calling the rcl function.
        """
        with self.handle:
            self.__subscription.set_content_filter(filter_expression, expression_parameters)

    def get_content_filter(self) -> ContentFilterOptions:
        """
        Get the filter expression and expression parameters for the subscription.

        :return: ContentFilterOptions object containing the filter expression and expression
            parameters.
        :raises: RCLError if internal error occurred when calling the rcl function.
        """
        with self.handle:
            return self.__subscription.get_content_filter()
