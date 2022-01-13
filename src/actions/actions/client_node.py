from __future__ import annotations

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import ThreadPoolExecutor

from actions import ServerNode, ClientGoalHandleSpec
from interfaces.action import Wait


class ClientNode(Node):

    _action_future: Future | None = None

    def __init__(self):
        super().__init__('action_client')

        self.get_logger().info('>> __init__')

        self._action_client: ActionClient = ActionClient(self, Wait, ServerNode.Action.WAIT)

        self.get_logger().info('<< __init__')

    def run(self, count: int = 10, timeout: float = 1.0):

        self.get_logger().info('>> run')

        goal = Wait.Goal()
        goal.count = count
        goal.timeout = timeout

        self._action_client.wait_for_server()

        def feedback_callback(feedback_msg):
            self.get_logger().info("-- feedback_callback")
            feedback: Wait.Wait_Feedback = feedback_msg.feedback
            self.get_logger().info(f"   feedback: {feedback.index=}")

        def result_callback(future: Future):
            self.get_logger().info("-- result_callback")

            goal_result = future.result()
            result: Wait.Result = goal_result.result

            if goal_result.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("   result: STATUS_ABORTED")
            elif goal_result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f"   result: STATUS_CANCELED")
                self._action_future = None
            else:
                self.get_logger().info(f"   result: {result.end_count=}")
                self._action_future = None

        def response_callback(future: Future):
            self.get_logger().info("-- response_callback")

            goal_request = future.result()
            if not goal_request.accepted:
                self.get_logger().info(f"   request is not accepted")
                self._action_future = None
                return

            self.get_logger().info(f"   request is accepted")
            get_result_future = goal_request.get_result_async()
            get_result_future.add_done_callback(result_callback)

        self._action_future = self._action_client.send_goal_async(goal, feedback_callback=feedback_callback)
        self._action_future.add_done_callback(response_callback)

        self.get_logger().info('<< run')

    def cancel(self):
        self.get_logger().info('>> cancel')

        goal = self._action_future.result()
        goal.cancel_goal()

        self.get_logger().info('<< cancel')


thread_pool = ThreadPoolExecutor(10)


def _cancel(node: ClientNode):
    time.sleep(6)
    node.cancel()
    time.sleep(3)
    node.run()


def _rerun(node: ClientNode):
    time.sleep(3)
    node.run()


def main(args=None):
    rclpy.init(args=args)

    node = ClientNode()

    node.run()
    thread_pool.submit(_rerun, node)
    thread_pool.submit(_cancel, node)

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
