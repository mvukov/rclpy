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

import rclpy.executors
from rclpy.utilities import get_rmw_implementation_identifier
from test_msgs.srv import BasicTypes

import rclpy

# TODO(sloretz) Reduce fudge once wait_for_service uses node graph events
TIME_FUDGE = 0.3


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

    def test_service_introspection_nomincal(self):
        srv = self.node.create_service(BasicTypes, 'test_service', callback = lambda req, resp: resp)
        cli = self.node.create_client(BasicTypes, 'test_service')
        sub = self.node.create_subscription(BasicTypes.Event, 'test_service/_service_event',
                                            lambda msg: print(msg), 10)

        req = BasicTypes.Request()
        req.bool_value = False
        req.int64_value = 12345

        resp = cli.call(req)

if __name__ == '__main__':
    unittest.main()
