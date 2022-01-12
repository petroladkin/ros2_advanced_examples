

class ServerGoalHandleSpec:
    """Spec for ServerGoalHandle class from /opt/ros/galactic/lib/python3.8/site-packages/rclpy/action/server.py"""

    @property
    def request(self):
        return None

    @property
    def goal_id(self):
        return None

    @property
    def is_active(self):
        return False

    @property
    def is_cancel_requested(self):
        return False

    @property
    def status(self):
        return None

    def execute(self, execute_callback=None):
        pass

    def publish_feedback(self, feedback):
        pass

    def succeed(self):
        pass

    def abort(self):
        pass

    def canceled(self):
        pass

    def destroy(self):
        pass


class ClientGoalHandleSpec:
    """Spec for ClientGoalHandle class from /opt/ros/galactic/lib/python3.8/site-packages/rclpy/action/client.py"""

    @property
    def goal_id(self):
        return None

    @property
    def stamp(self):
        return None

    @property
    def accepted(self):
        return None

    @property
    def status(self):
        return None

    def cancel_goal(self):
        pass

    def cancel_goal_async(self):
        pass

    def get_result(self):
        pass

    def get_result_async(self):
        pass
