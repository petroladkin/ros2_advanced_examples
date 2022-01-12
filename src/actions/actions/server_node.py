from __future__ import annotations

import time
import rclpy
import threading

from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from actions import ServerGoalHandleSpec
from interfaces.action import Wait


class ServerNode(Node):

    class Action(str, Enum):
        WAIT = 'action/wait'

    _need_to_cancel: bool = False
    _goal_handle: ServerGoalHandleSpec | None = None
    _goal_lock = threading.Lock()
    _execute_lock = threading.Lock()

    def __init__(self):
        super().__init__('action_server')

        self.get_logger().info('>> __init__')

        self._act_server = ActionServer(
            self,
            Wait,
            ServerNode.Action.WAIT,
            execute_callback=self.__exec_cb,
            goal_callback=self.__goal_cb,
            handle_accepted_callback=self.__accepted_cb,
            cancel_callback=self.__cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('<< __init__')

    def __goal_cb(self, goal_handle: ServerGoalHandleSpec):
        """ callback called when incoming goal request """
        self.get_logger().info(f'')
        self.get_logger().info(f'>> __goal_cb')
        self.get_logger().info(f'<< __goal_cb')

        # if to need reject request for new actions then return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def __cancel_cb(self, goal_handle: ServerGoalHandleSpec):
        """ callback called when incoming call to cancel running goal """
        self.get_logger().info(f'<{id(goal_handle)}>: >> __cancel_cb')

        with self._goal_lock:
            # set flag to cancel current action
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info(f'<{id(goal_handle)}>:   set cancel flag for goal <{id(self._goal_handle)}>')
                self._need_to_cancel = True

        self.get_logger().info(f'<{id(goal_handle)}>: << __cancel_cb')

        return CancelResponse.ACCEPT

    def __accepted_cb(self, goal_handle: ServerGoalHandleSpec):
        """
            callback called after accepted incoming call in self.__goal_cb

            !!!  in this callback YOU MUST CALL goal_handle.execute() to run action callback  !!!
        """
        self.get_logger().info(f'<{id(goal_handle)}>: >> __accepted_cb')

        with self._goal_lock:
            # if running previous action then need cancel it
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info(f'<{id(goal_handle)}>:   abort goal <{id(self._goal_handle)}>')
                self._goal_handle.abort()

            # save goal handle and reset cancel flag
            self._need_to_cancel = False
            self._goal_handle = goal_handle

            goal_handle.execute()

        self.get_logger().info(f'<{id(goal_handle)}>: << __accepted_cb>')

    def __exec_cb(self, goal_handle: ServerGoalHandleSpec):
        """ callback to execute action """
        self.get_logger().info(f'<{id(goal_handle)}>: >> __exec_cb')

        with self._execute_lock:
            result = Wait.Result()

            incoming_goal: Wait.Goal = goal_handle.request
            self.get_logger().info(f'<{id(goal_handle)}>:    {incoming_goal.timeout=}')
            self.get_logger().info(f'<{id(goal_handle)}>:    {incoming_goal.count=}')

            feedback = Wait.Feedback()
            for i in range(incoming_goal.count):
                self.get_logger().info(f'<{id(goal_handle)}>:   step: {i}')
                feedback.index = i
                goal_handle.publish_feedback(feedback)
                time.sleep(incoming_goal.timeout)

                result.end_count = i

                # to cancel if set cancel flag
                if self._need_to_cancel:
                    self.get_logger().info(f'<{id(goal_handle)}>:   need to cancel [self._need_to_cancel]')
                    goal_handle.canceled()
                    self.get_logger().info(f'<{id(goal_handle)}>: << __exec_cb [canceled]')
                    return result

                # to exit if goal aborted (see self.__accepted_cb)
                if not goal_handle.is_active:
                    self.get_logger().info(f'<{id(goal_handle)}>:   need to cancel [not is_active]')
                    self.get_logger().info(f'<{id(goal_handle)}>: << __exec_cb [aborted]')
                    return result

            # normal exit
            goal_handle.succeed()

            self.get_logger().info(f'<{id(goal_handle)}>: << __exec_cb [normal]]')
            return result


def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
