"""Mission Controller
Description:
    This ros2 node commands the robot to bounce around until timeout.
License:
    Copyright 2021 Debby Nirwan
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""
import rclpy

from rclpy.node import Node
from std_srvs.srv import SetBool

MINUTE = 60
DEFAULT_MAPPING_TIME = 5


class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')
        self._start_bouncing_srv = self.create_client(SetBool,
                                                      '/start_stop_bouncing')
        self._mission_time = self.declare_parameter('mission_time',
                                                    DEFAULT_MAPPING_TIME)
        self._bounce = True
        self.timer = self.create_timer(self._mission_time.value * MINUTE,
                                       self.timer_callback)

        while not self._start_bouncing_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetBool.Request()
        request.data = self._bounce
        self._bounce = not self._bounce
        self._start_bouncing_srv.call_async(request)

    def timer_callback(self):
        request = SetBool.Request()
        request.data = self._bounce
        self._bounce = not self._bounce
        self._start_bouncing_srv.call_async(request)
        self.timer.cancel()
        self.get_logger().info("mission completed")


def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionController()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
