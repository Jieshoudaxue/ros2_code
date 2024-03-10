import rclpy 
from rclpy.action import ActionClient
from rclpy.node import Node

from diy_interface.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(goal_msg,
                                feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Sent goal request: {0}'.format(order))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal rejected :(")
            return
        
        self.get_logger().info("goal accepted :)")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_client = FibonacciActionClient()
    fibonacci_action_client.send_goal(10)

    rclpy.spin(fibonacci_action_client)

    fibonacci_action_client.destroy_node()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()