from __future__ import annotations

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.logging import LoggingSeverity
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from interfaces.action import For
from concurrent.futures import ThreadPoolExecutor


class ClientNode(Node):

    def __init__(self):
        super().__init__('action_client')

        self._action_future: Future | None = None
        self._get_result_future: Future | None = None
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.get_logger().debug(f'__init__')

        self._action_client: ActionClient = ActionClient(self, For, 'for')

    def run(self):

        self.get_logger().debug(f'run')

        goal = For.Goal()
        goal.count = 10

        self._action_client.wait_for_server()

        def feedback_callback(feedback_msg):
            self.get_logger().debug(f"current index: {feedback_msg.feedback.index}")

        def result_callback(future: Future):
            self.get_logger().debug(f"result_callback: {future}")

            goal_result = future.result()
            result: For.Result = goal_result.result

            if goal_result.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().debug(f"result: STATUS_ABORTED")
            elif goal_result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().debug(f"result: STATUS_CANCELED")
            else:
                self.get_logger().debug(f"result {result.result}")

            self._action_future = None
            self._action_get_result_future = None
            self._goal_response = None

        def response_callback(future: Future):
            self.get_logger().debug(f"response_callback: {future}")

            self._goal_response = future.result()
            if not self._goal_response.accepted:
                self._action_future = None
                self._action_get_result_future = None
                self._goal_response = None
                return

            self._get_result_future = self._goal_response.get_result_async()
            self._get_result_future.add_done_callback(result_callback)

        self._action_future = self._action_client.send_goal_async(goal, feedback_callback=feedback_callback)
        self._action_future.add_done_callback(response_callback)

    def cancel(self):
        self.get_logger().info('Need to cancel')

        goal = self._action_future.result()
        goal.cancel_goal()


def _cancel(action_client: ActionClientNode):
    time.sleep(3)
    action_client.cancel()


def _rerun(action_client: ActionClientNode):
    time.sleep(3)
    action_client.run()


def main(args=None):
    rclpy.init(args=args)

    thread_pool = ThreadPoolExecutor(2)
    action_client = ActionClientNode()

    action_client.run()

    # thread_pool.submit(_rerun, action_client)
    thread_pool.submit(_cancel, action_client)

    rclpy.spin(action_client, executor=MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
