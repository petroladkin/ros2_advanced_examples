import rclpy

from enum import Enum
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType


class ParametersNode(Node):

    class Parameter(str, Enum):
        DEBUG_LOGGER = 'debug_logger'
        READ_ON_START_PARAM = 'read_on_start_param'
        MONITORING_PARAM = 'monitoring_param'

    _debug_logger: bool = False
    _read_on_start_param: int = 0
    _monitoring_param: str = "default"

    def __init__(self):
        super().__init__('parameters')

        self.get_logger().info('>> __init__')

        # register callback to monitoring parameters change
        self.add_on_set_parameters_callback(self._update_parameters_callback)

        # declare parameters
        self.declare_parameter(
            ParametersNode.Parameter.DEBUG_LOGGER, self._debug_logger,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Enable DEBUG level in logger"))

        self.declare_parameter(
            ParametersNode.Parameter.READ_ON_START_PARAM, self._read_on_start_param,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="This param read only on start node"))

        self.declare_parameter(
            ParametersNode.Parameter.MONITORING_PARAM, self._monitoring_param,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="This param monitoring to change"))

        # if You don't monitor the parameter in '_update_parameters_callback'
        # then you need read the parameter value before using it
        value = self.get_parameter(ParametersNode.Parameter.READ_ON_START_PARAM).get_parameter_value()
        self._read_on_start_param = value.integer_value
        self.get_logger().info(f'  did get read_on_start_param parameter: "{self._read_on_start_param}"')

        self.get_logger().debug(f'  INIT WITH:')
        self.get_logger().debug(f'    {self._debug_logger=}')
        self.get_logger().debug(f'    {self._read_on_start_param=}')
        self.get_logger().debug(f'    {self._monitoring_param=}')

        self.get_logger().info('<< __init__')

    def _update_parameters_callback(self, params):
        """ update parameters callback """
        self.get_logger().info('>> _update_parameters_callback')

        for param in params:
            self.get_logger().info(f'  {param.name}={param.value} [{param.type_}]')
            if param.name == ParametersNode.Parameter.DEBUG_LOGGER and param.type_ == Parameter.Type.BOOL:
                self._debug_logger = bool(param.value)
                self.get_logger().set_level(LoggingSeverity.DEBUG if self._debug_logger else LoggingSeverity.INFO)
                self.get_logger().debug(f'  UPDATED debug_logger to "{param.value}"')
            elif param.name == ParametersNode.Parameter.MONITORING_PARAM and param.type_ == Parameter.Type.STRING:
                self._monitoring_param = str(param.value)
                self.get_logger().debug(f'  UPDATED monitoring_param to "{param.value}"')

        self.get_logger().info('<< _update_parameters_callback')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ParametersNode()
    try:
        rclpy.spin(node, executor=SingleThreadedExecutor())
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
