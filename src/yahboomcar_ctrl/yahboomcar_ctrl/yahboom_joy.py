#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import rclpy
from rclpy.node import Node
import getpass
import threading
from time import sleep
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalInfo


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('yahboom_joy')
        self.Joy_active = False
        self.Buzzer_active = False
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_Gear = 1
        self.angular_Gear = 1
        self.rate_hz = 20  # Will use time.sleep instead of rospy.Rate
        
        # Declare parameters
        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 1.0)
        
        # Get parameter values
        self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        # Publishers
        self.pub_goal = self.create_publisher(GoalInfo, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight = self.create_publisher(Int32, "RGBLight", 10)
        
        # Subscriber
        self.sub_Joy = self.create_subscription(Joy, 'joy', self.buttonCallback, 10)

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        # print ("user_name: ", self.user_name)
        # print ("joy_data.axes: ", joy_data.axes)
        # print ("joy_data.buttons: ", joy_data.buttons)
        if self.user_name == "jetson": self.user_jetson(joy_data)
        else: self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        '''
        :jetson joy_data:
            axes 8: [0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
            左摇杆(左正右负): axes[0]
            左摇杆(上正下负): axes[1]
            右摇杆(左正右负): axes[2]
            右摇杆(上正下负): axes[3]
            R2(按负抬正): axes[4]
            L2(按负抬正): axes[5]
            左按键(左正右负): axes[6]
            左按键(上正下负): axes[7]
            buttons 15:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            A: buttons[0]
            B: buttons[1]
            X: buttons[3]
            Y: buttons[4]
            L1: buttons[6]
            R1: buttons[7]
            SELECT: buttons[10]
            START: buttons[11]
            左摇杆按下: buttons[13]
            右摇杆按下: buttons[14]
        '''
        # Cancel
        if joy_data.axes[4] == -1: self.cancel_nav()
        # RGBLight
        if joy_data.buttons[7] == 1:
            if self.RGBLight_index < 6:
                for i in range(3): self.pub_RGBLight.publish(Int32(data=self.RGBLight_index))
                # print ("pub RGBLight success")
            else: self.RGBLight_index = 0
            self.RGBLight_index += 1
        # Buzze
        if joy_data.buttons[11] == 1:
            self.Buzzer_active=not self.Buzzer_active
            # print "self.Buzzer_active: ", self.Buzzer_active
            for i in range(3): self.pub_Buzzer.publish(Bool(data=self.Buzzer_active))
        # linear Gear control
        if joy_data.buttons[13] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        # angular Gear control
        if joy_data.buttons[14] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 0.2
            elif self.angular_Gear == 0.2: self.angular_Gear = 0.4
            elif self.angular_Gear == 0.4: self.angular_Gear = 0.8
            elif self.angular_Gear == 0.8: self.angular_Gear = 1.0
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        #ylinear_speed = self.filter_data(joy_data.axes[2]) * self.yspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[2]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for i in range(3): self.pub_cmdVel.publish(twist)

    def user_pc(self, joy_data):
        '''
        :pc joy_data:
            axes 8: [ -0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0 ]
            左摇杆左正右负: axes[0]
            左摇杆上正下负: axes[1]
            L2按负抬正:  axes[2]
            右摇杆左正右负: axes[3]
            右摇杆上正下负: axes[4]
            R2按负抬正:  axes[5]
            左按键左正右负: axes[6]
            左按键上正下负: axes[7]
            buttons 11: [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
            A: buttons[0]
            B: buttons[1]
            X: buttons[2]
            Y: buttons[3]
            L1: buttons[4]
            R1: buttons[5]
            SELECT: buttons[6]
            MODE: buttons[7]
            START: buttons[8]
            左摇杆按下: buttons[9]
            右摇杆按下: buttons[10]
        '''
        # print ("joy_data.buttons: ", joy_data.buttons)
        # print ("joy_data.axes: ", joy_data.axes)
        # 取消 Cancel
        if joy_data.axes[5] == -1: self.cancel_nav()
        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6:
                self.pub_RGBLight.publish(Int32(data=self.RGBLight_index))
                # print ("pub RGBLight success")
            else: self.RGBLight_index = 0
            self.RGBLight_index += 1
        if joy_data.buttons[7] == 1:
            self.Buzzer_active=not self.Buzzer_active
            # print "self.Buzzer_active: ", self.Buzzer_active
            self.pub_Buzzer.publish(Bool(data=self.Buzzer_active))
        # 档位控制 Gear control
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[3]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for i in range(3): self.pub_cmdVel.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2: value = 0
        return value

    def cancel_nav(self):
        # 发布move_base取消命令
        # Issue the move_base cancel command
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            for i in range(3):
                self.pub_JoyState.publish(Bool(data=self.Joy_active))
                self.pub_goal.publish(GoalInfo())
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time

    def cancel(self):
        # In ROS2, publishers/subscribers are cleaned up automatically
        self.get_logger().info("Shutting down joystick teleop...")
        # Stop robot movement
        self.pub_cmdVel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    
    try:
        joy = JoyTeleop()
        rclpy.spin(joy)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'joy' in locals():
            joy.cancel()
        rclpy.shutdown()

if __name__ == '__main__':
    main()