import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from diy_interface.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info("executing goal...")

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("pub feedback")
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()