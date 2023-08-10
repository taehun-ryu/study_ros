# !/usr/bin/env/ python3
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from study_msgs.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__("fibonacci_action_client")
				# Server에서 지정한 action 이름과 일치해야 한다는 점에 유의하세요.
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.get_logger().info("=== Fibonacci Action Client Started ====")

		# client 생성 시 callback으로 묶이는 것이 아니기 때문에, 직접 main에서 호출해야 합니다.
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

				# 10초간 server를 기다리다가 응답이 없으면 에러를 출력합니다. 
        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error("Server Not exists")

				# goal request가 제대로 보내졌는지 알기 위해 future가 사용됩니다.
				# 더불어, feedback_callback을 묶어 feedback 발생 시 해당 함수로 이동합니다.
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

				# server가 존재한다면, Goal Request의 성공 유무, 
				# 최종 Result에 대한 callback도 필요합니다.
        self._send_goal_future.add_done_callback(self.goal_response_callback)

		# feedback을 받아오고, 지금은 단순히 출력만 합니다.
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"Received feedback: {feedback.partial_sequence}")

		# Goal Request에 대한 응답 시 실행될 callback입니다.
    def goal_response_callback(self, future):
        goal_handle = future.result()

				# Goal type에 따라 성공 유무를 판단합니다.
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
				
				# 아직 callback이 남았습니다!
				# 만약 최종 Result 데이터를 다룰 callback을 연동합니다.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

		# Result callback은 future를 매개변수로 받습니다.
		# future내에서 result에 접근하는 과정에 유의하시기 바랍니다.
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().warn(f"Action Done !! Result: {result.sequence}")
        rclpy.shutdown()

    
def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_client = FibonacciActionClient()
		
		# Client Node 생성 이후 직접 send_goal을 해줍니다. Service와 유사하지요
		# 지금은 딱히 작업이 없지만 Goal Request에 대한 future를 반환하도록 해두었습니다. 
    future = fibonacci_action_client.send_goal(5)

    rclpy.spin(fibonacci_action_client)


if __name__ == "__main__":
    main()