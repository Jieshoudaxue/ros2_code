#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import sys, select, termios, tty

class MbotTeletop(Node):
    def __init__(self, name):
        super().__init__(name)

        self._help_msg = """
Control mbot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

        self._move_bindings = {
            'i':(1,0),
            'o':(1,-1),
            'j':(0,1),
            'l':(0,-1),
            'u':(1,1),
            ',':(-1,0),
            '.':(-1,1),
            'm':(-1,-1),
            }
        
        self._speed_bindings = {
            'q':(1.1,1.1),
            'z':(.9,.9),
            'w':(1.1,1),
            'x':(.9,1),
            'e':(1,1.1),
            'c':(1,.9),
            }

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key

    def run(self):
        x = 0
        th = 0
        status = 0
        count = 0
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0
        
        speed = .2
        turn = 1        

        try:
            print(self._help_msg)
            print("currently:\tspeed %s\tturn %s " % (speed,turn))
            while(1):
                key = self.getKey()
                
                # 运动控制方向键（1：正方向，-1负方向）
                if key in self._move_bindings.keys():
                    x = self._move_bindings[key][0]
                    th = self._move_bindings[key][1]
                    count = 0
                # 速度修改键
                elif key in self._speed_bindings.keys():
                    speed = speed * self._speed_bindings[key][0]  # 线速度增加0.1倍
                    turn = turn * self._speed_bindings[key][1]    # 角速度增加0.1倍
                    count = 0

                    print("currently:\tspeed %s\tturn %s " % (speed,turn))
                    if (status == 14):
                        print (self._help_msg)
                    status = (status + 1) % 15
                # 强制停止键
                elif key == ' ' or key == 'k' :
                    x = 0
                    th = 0
                    control_speed = 0
                    control_turn = 0
                    print("currently:\tforce stop !!")
                # 其他键都是慢慢停止键
                else:
                    count = count + 1
                    if count > 4:
                        x = 0
                        th = 0
                    if (key == '\x03'):
                        break

                # 目标速度=速度值*方向值
                target_speed = speed * x
                target_turn = turn * th

                # 速度限位，防止速度增减过快
                if target_speed > control_speed:
                    control_speed = min( target_speed, control_speed + 0.02 )
                elif target_speed < control_speed:
                    control_speed = max( target_speed, control_speed - 0.02 )
                else:
                    control_speed = target_speed

                # 角度限位，防止角度增减过快
                if target_turn > control_turn:
                    control_turn = min( target_turn, control_turn + 0.1 )
                elif target_turn < control_turn:
                    control_turn = max( target_turn, control_turn - 0.1 )
                else:
                    control_turn = target_turn

                # 创建并发布twist消息
                twist = Twist()
                twist.linear.x = float(control_speed)
                twist.linear.y = float(0)
                twist.linear.z = float(0)
                twist.angular.x = float(0)
                twist.angular.y = float(0)
                twist.angular.z = float(control_turn)
                self._cmd_pub.publish(twist)

        except Exception as e: 
            print(e)
        finally:
            twist = Twist()
            self._cmd_pub.publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)        

def main(args=None):
    rclpy.init(args=args)

    node = MbotTeletop("Tele_cmd")
    node.run()

    rclpy.shutdown()
    


if __name__ == "__main__":
  main()
