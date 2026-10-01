import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__('simple_parameter')
        
        
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "default_string")
        
        self.add_on_set_parameters_callback(self.paramChangeCallback)
        
    def paramChangeCallback(self, params):
        result = SetParametersResult()
        for param in params:
            if param.name == "simple_int_param":
                if param.value < 0:
                    result.successful = False
                    result.reason = "simple_int_param must be non-negative"
                    return result
            elif param.name == "simple_string_param":
                if len(param.value) > 20:
                    result.successful = False
                    result.reason = "simple_string_param must be 20 characters or less"
                    return result
        result.successful = True
        return result


def main():
    rclpy.init()
    node = SimpleParameter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    