# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from typing import List
import unittest

import rclpy
import rclpy.executors
from rclpy.parameter import Parameter
from service_msgs.msg import ServiceEventInfo
from test_msgs.srv import BasicTypes


class TestServiceEvents(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            'TestClient', context=self.context, enable_service_introspection=True)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.srv = self.node.create_service(BasicTypes, 'test_service', self.srv_callback)
        self.cli = self.node.create_client(BasicTypes, 'test_service')
        self.sub = self.node.create_subscription(BasicTypes.Event, 'test_service/_service_event',
                                                 self.sub_callback, 10)
        self.event_messages: List[BasicTypes.Event] = []

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def sub_callback(self, msg):
        self.event_messages.append(msg)

    def srv_callback(self, req, resp):
        resp.bool_value = not req.bool_value
        resp.int64_value = req.int64_value
        return resp

    def test_service_introspection_nominal(self):
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)

        # Wait for the service event messages to be published (this screams flaky...)
        for _ in range(10):
            self.executor.spin_once(0.1)

        self.assertEqual(len(self.event_messages), 4)
        result_dict = {}
        for msg in self.event_messages:
            result_dict[msg.info.event_type] = msg
        self.assertEqual(
            set(result_dict.keys()),
            {ServiceEventInfo.REQUEST_SENT, ServiceEventInfo.REQUEST_RECEIVED,
             ServiceEventInfo.RESPONSE_SENT, ServiceEventInfo.RESPONSE_RECEIVED})
        self.assertEqual(result_dict[ServiceEventInfo.REQUEST_SENT].request[0].int64_value, 12345)
        self.assertEqual(result_dict[ServiceEventInfo.REQUEST_SENT].request[0].bool_value, False)
        self.assertEqual(result_dict[ServiceEventInfo.RESPONSE_SENT].response[0].bool_value, True)
        self.assertEqual(
            result_dict[ServiceEventInfo.RESPONSE_RECEIVED].response[0].int64_value, 12345)

    def test_enable_disable_service_events(self):
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        self.node.set_parameters([
            Parameter('publish_service_events', Parameter.Type.BOOL, False),
            Parameter('publish_client_events', Parameter.Type.BOOL, False)])
        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)
        for _ in range(10):
            self.executor.spin_once(0.1)
        self.assertEqual(len(self.event_messages), 0)

        self.event_messages = []
        result_dict = {}
        self.node.set_parameters([
            Parameter('publish_service_events', Parameter.Type.BOOL, True),
            Parameter('publish_client_events', Parameter.Type.BOOL, False)])
        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)
        for _ in range(10):
            self.executor.spin_once(0.1)
        self.assertEqual(len(self.event_messages), 2)
        for msg in self.event_messages:
            result_dict[msg.info.event_type] = msg
        self.assertEqual(
            set(result_dict.keys()),
            {ServiceEventInfo.REQUEST_RECEIVED, ServiceEventInfo.RESPONSE_SENT})
        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_RECEIVED].response), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_SENT].response), 1)
        self.assertEqual(result_dict[ServiceEventInfo.RESPONSE_SENT].response[0].bool_value, True)

        self.event_messages = []
        result_dict = {}
        self.node.set_parameters([
            Parameter('publish_service_events', Parameter.Type.BOOL, False),
            Parameter('publish_client_events', Parameter.Type.BOOL, True)])
        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)
        for _ in range(10):
            self.executor.spin_once(0.1)
        self.assertEqual(len(self.event_messages), 2)
        for msg in self.event_messages:
            result_dict[msg.info.event_type] = msg
        self.assertEqual(
            set(result_dict.keys()),
            {ServiceEventInfo.REQUEST_SENT, ServiceEventInfo.RESPONSE_RECEIVED})
        self.assertEqual(len(result_dict[ServiceEventInfo.REQUEST_SENT].response), 0)
        self.assertEqual(len(result_dict[ServiceEventInfo.RESPONSE_RECEIVED].response), 1)
        self.assertEqual(result_dict[ServiceEventInfo.REQUEST_SENT].request[0].bool_value, False)

    def test_enable_disable_service_event_payload(self):
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        self.node.set_parameters([
            Parameter('publish_service_content', Parameter.Type.BOOL, False),
            Parameter('publish_client_content', Parameter.Type.BOOL, False)])
        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)
        for _ in range(10):
            self.executor.spin_once(0.1)
        self.assertEqual(len(self.event_messages), 4)
        for i in self.event_messages:
            self.assertEqual(len(i.request), 0)
            self.assertEqual(len(i.response), 0)

        self.event_messages = []
        self.node.set_parameters([
            Parameter('publish_service_content', Parameter.Type.BOOL, True),
            Parameter('publish_client_content', Parameter.Type.BOOL, False)])
        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)
        for _ in range(10):
            self.executor.spin_once(0.1)
        self.assertEqual(len(self.event_messages), 4)
        for i in self.event_messages:
            if i.info.event_type == ServiceEventInfo.REQUEST_RECEIVED:
                self.assertEqual(len(i.request), 1)
            elif (i.info.event_type == ServiceEventInfo.RESPONSE_SENT):
                self.assertEqual(len(i.response), 1)
            else:
                self.assertEqual(len(i.request) + len(i.response), 0)

        self.event_messages = []
        self.node.set_parameters([
            Parameter('publish_service_content', Parameter.Type.BOOL, False),
            Parameter('publish_client_content', Parameter.Type.BOOL, True)])
        future = self.cli.call_async(req)
        self.executor.spin_until_future_complete(future)
        for _ in range(10):
            self.executor.spin_once(0.1)
        self.assertEqual(len(self.event_messages), 4)
        for i in self.event_messages:
            if i.info.event_type == ServiceEventInfo.REQUEST_SENT:
                self.assertEqual(len(i.request), 1)
            elif (i.info.event_type == ServiceEventInfo.RESPONSE_RECEIVED):
                self.assertEqual(len(i.response), 1)
            else:
                self.assertEqual(len(i.request) + len(i.response), 0)


if __name__ == '__main__':
    unittest.main()
