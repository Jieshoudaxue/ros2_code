import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

MAX_RANGE = 0.15

# ObstacleAvoider 是机器人行动的控制程序，主要是避障，防止机器人撞墙
# 他创建了一个 ros2 节点 obstacle_avoider，订阅了两个传感器话题 left_sensor 和 right_sensor，
# 接收 Range 消息，然后根据传感器的距离，通过 cmd_vel 发给 mbot_driver 控制机器人运动
class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self._publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'left_sensor', self._left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self._right_sensor_callback, 1)

    def _left_sensor_callback(self, message):
        self._left_sensor_value = message.range

    def _right_sensor_callback(self, message):
        self._right_sensor_value = message.range

        # 每次收到一个传感器的数据，就计算一次机器人的运动控制
        command_message = Twist()
        command_message.linear.x = 0.1
        # 如果左右传感器的距离有一个小于 0.9 * MAX_RANGE，就让机器人向顺时针向右转，否则默认为0，即直线行驶
        if self._left_sensor_value < 0.9 * MAX_RANGE or self._right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0

        self._publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()