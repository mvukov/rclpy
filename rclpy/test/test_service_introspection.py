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

import platform
import time
import unittest

import rclpy
import rclpy.executors
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from test_msgs.srv import BasicTypes
from service_msgs.msg import ServiceEventInfo

import rclpy


class TestClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            'TestClient', context=cls.context, enable_service_introspection=True)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_service_introspection_nominal(self):
        event_messages = []

        def callback(msg):
            event_messages.append(msg)

        def srv_callback(req, resp):
            resp.bool_value = not req.bool_value
            resp.int64_value = req.int64_value
            return resp

        srv = self.node.create_service(BasicTypes, 'test_service', callback=srv_callback)
        cli = self.node.create_client(BasicTypes, 'test_service')
        sub = self.node.create_subscription(BasicTypes.Event, 'test_service/_service_event',
                                            callback, 10)
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        future = cli.call_async(req)
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self.node, future, executor=executor)
        
        # Wait for the service event to be published (this screams flaky...)
        # for i in range(10):
        rclpy.spin_once(self.node, executor=executor)
        rclpy.spin_once(self.node, executor=executor)
        rclpy.spin_once(self.node, executor=executor)
        rclpy.spin_once(self.node, executor=executor)
        rclpy.spin_once(self.node, executor=executor)
        rclpy.spin_once(self.node, executor=executor)
        rclpy.spin_once(self.node, executor=executor)

        self.assertEqual(len(event_messages), 4)
        self.assertEqual(event_messages[0].info.event_type, ServiceEventInfo.REQUEST_SENT)
        self.assertEqual(event_messages[1].info.event_type, ServiceEventInfo.REQUEST_RECEIVED)
        self.assertEqual(event_messages[2].info.event_type, ServiceEventInfo.RESPONSE_SENT)
        self.assertEqual(event_messages[3].info.event_type, ServiceEventInfo.RESPONSE_RECEIVED)
        self.assertEqual(event_messages[0].request[0].bool_value, False)
        self.assertEqual(event_messages[0].request[0].int64_value, 12345)
        self.assertEqual(event_messages[3].response[0].bool_value, True)
        self.assertEqual(event_messages[3].response[0].int64_value, 12345)

    def test_enable_disable_service_events(self):
        event_messages = []

        def callback(msg):
            event_messages.append(msg)

        def srv_callback(req, resp):
            resp.bool_value = not req.bool_value
            resp.int64_value = req.int64_value
            return resp

        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        srv = self.node.create_service(BasicTypes, 'test_service', callback=srv_callback)
        cli = self.node.create_client(BasicTypes, 'test_service')
        sub = self.node.create_subscription(BasicTypes.Event, 'test_service/_service_event',
                                            callback, 10)
        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        self.node.set_parameters([
            Parameter('publish_service_events', Parameter.Type.BOOL, False),
            Parameter('publish_client_events', Parameter.Type.BOOL, False), ])

        future = cli.call_async(req)
        for i in range(4):
            rclpy.spin_once(self.node, executor=executor)
        self.assertEqual(len(event_messages), 2)



    def test_enable_disable_service_event_payload(self):
        pass



if __name__ == '__main__':
    unittest.main()
