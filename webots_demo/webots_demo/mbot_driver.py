import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# MbotDriver 是机器人的控制器程序，他创建了一个 ros2 节点 mbot_driver，
# 订阅 cmd_vel 话题，接收 Twist 消息，然后根据 Twist 消息的线速度和角速度控制机器人的左右轮速度，从而实现机器人的运动控制。
class MbotDriver:
    def init(self, webots_node, properties):
        # 获取 webots 里的 robot 对象，在本样例就是 my_world.wbt 中的 mbot_car
        self._robot = webots_node.robot

        # 获取左右轮的电机对象，并设置电机的目标位置为极大（一直旋转）和速度为 0
        self._left_motor = self._robot.getDevice('left wheel motor')
        self._right_motor = self._robot.getDevice('right wheel motor')

        self._left_motor.setPosition(float('inf'))
        self._left_motor.setVelocity(0)

        self._right_motor.setPosition(float('inf'))
        self._right_motor.setVelocity(0)

        self._target_twist = Twist()

        # 创建一个 ros2 节点 mbot_driver，订阅 cmd_vel 话题，用来驱动 mbot_car
        # 接收到的 Twist 消息存入 self._target_twist 中，等待 step 函数处理
        rclpy.init(args=None)
        self._node = rclpy.create_node('mbot_driver')
        self._node.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 1)

    def _cmd_vel_callback(self, twist):
        self._target_twist = twist

    # step 由 webots_ros2_driver.webots_controller.WebotsController 调用，称之为 simulation step
    # 这里可以理解为是机器人控制器的主循环函数，周期调用
    def step(self):
        # 使用 spin_once 来处理 mbot_driver 的一次事件，这里是一次 cmd_vel 订阅
        # 如果没有这个函数，_cmd_vel_callback 是不会被执行的
        # 如果事件没来，会立即返回，不会阻塞，确保实时性
        rclpy.spin_once(self._node, timeout_sec=0)

        # 这里讲解了 Twist 理解 和 右手定则：https://blog.csdn.net/cy1641395022/article/details/131236155
        # 获取机器人的前进速度和旋转速度，根据右手定则：
        # 如果机器人向左转（顺时针），angular_speed为负；
        # 如果机器人向右转（逆时针），angular_speed为正；
        forward_speed = self._target_twist.linear.x
        angular_speed = self._target_twist.angular.z

        # 机器人的前进和旋转速度需要转换为左右轮转速，由于机器人是差速驱动，所以需要根据机器人的轮距和轮径来计算左右轮转速
        # HALF_DISTANCE_BETWEEN_WHEELS 是机器人的轮距的一半，乘以 angular_speed 就是机器人内外轮线速度的补偿值
        # 得到内外轮线速度后，再除以 WHEEL_RADIUS 就是内外轮的转速（角速度乘以旋转半径为线速度）
        # 简单的三个场景，可以帮助理解这个公式：
        # 第一，控制机器人直线前进（forward_speed 为正，angular_speed 为0），左右轮转速必须相同，且转向相同
        # 第二，控制机器人原地顺时针旋转（forward_speed 为0，angular_speed 为负），左右轮转速必须相同，且转向相反
        # 第三，控制机器人原地逆时针旋转（forward_speed 为0，angular_speed 为正），左右轮转速必须相同，且转向相反
        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # 设置左右轮的目标转速
        self._left_motor.setVelocity(command_motor_left)
        self._right_motor.setVelocity(command_motor_right)