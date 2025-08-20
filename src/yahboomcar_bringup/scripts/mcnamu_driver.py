#!/usr/bin/env python3
# encoding: utf-8
import sys
import math
import rclpy
from rclpy.node import Node
import random
import threading
from math import pi
from time import sleep
from sensor_msgs.msg import Imu, MagneticField, JointState
from Rosmaster_Lib import Rosmaster
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
# TODO: Replace dynamic_reconfigure with ROS2 parameters later

car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}


class yahboomcar_driver(Node):
    def __init__(self):
        super().__init__('yahboomcar_driver')
        global car_type_dic
        # 弧度转角度
        # Radians turn angle
        self.RA2DE = 180 / pi
        self.car = Rosmaster()
        # Declare parameters with defaults
        self.declare_parameter('car_type', 'R2')
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('prefix', '')
        self.declare_parameter('xlinear_speed_limit', 1.0)
        self.declare_parameter('ylinear_speed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 1.0)
        self.declare_parameter('nav_use_rotvel', False)
        
        # Get parameter values
        self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
        self.car.set_car_type(5)  # R2L robot type
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.Prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.xlinear_limit = self.get_parameter('xlinear_speed_limit').get_parameter_value().double_value
        self.ylinear_limit = self.get_parameter('ylinear_speed_limit').get_parameter_value().double_value
        self.angular_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.nav_use_rotvel = self.get_parameter('nav_use_rotvel').get_parameter_value().bool_value
        # Subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.sub_RGBLight = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 100)
        self.sub_Buzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100)
        
        # Publishers
        self.EdiPublisher = self.create_publisher(Float32, 'edition', 100)
        self.volPublisher = self.create_publisher(Float32, 'voltage', 100)
        self.staPublisher = self.create_publisher(JointState, 'joint_states', 100)
        self.velPublisher = self.create_publisher(Twist, "pub_vel", 100)
        self.imuPublisher = self.create_publisher(Imu, "pub_imu", 100)
        self.magPublisher = self.create_publisher(MagneticField, "pub_mag", 100)
        # TODO: Add ROS2 parameter callback for dynamic reconfigure equivalent later
        self.car.create_receive_threading()
        self.rate_hz = 10  # Will use time.sleep instead of rospy.Rate

    def cancel(self):
        # In ROS2, publishers/subscribers are cleaned up automatically
        # Just need to stop the robot safely
        self.get_logger().info("Shutting down robot...")
        try:
            self.car.set_car_motion(0.0, 0.0, 0.0)  # Stop movement
            self.car.set_beep(0)  # Stop buzzer
            self.car.set_colorful_effect(0, 6, parm=1)  # Stop lights
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")
        sleep(1)

    def pub_data(self):
        # 发布小车运动速度、陀螺仪数据、电池电压
        ## Publish the speed of the car, gyroscope data, and battery voltage
        while rclpy.ok():
            #sleep(0.1)
            imu = Imu()
            twist = Twist()
            battery = Float32()
            edition = Float32()
            mag = MagneticField()
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = "joint_states"
            if len(self.Prefix)==0:
                state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
                              "front_right_steer_joint", "front_right_wheel_joint"]
            else:
                state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
                              self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
            edition.data = self.car.get_version()
            battery.data = self.car.get_battery_voltage()
            ax, ay, az = self.car.get_accelerometer_data()
            gx, gy, gz = self.car.get_gyroscope_data()
            mx, my, mz = self.car.get_magnetometer_data()
            vx, vy, angular = self.car.get_motion_data()
            
            # 发布陀螺仪的数据
            # Publish gyroscope data
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = self.imu_link
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            mag.header.stamp = self.get_clock().now().to_msg()
            mag.header.frame_id = self.imu_link
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = my
            mag.magnetic_field.z = mz
            # 将小车当前的线速度和角速度发布出去
            # Publish the current linear vel and angular vel of the car
            twist.linear.x = vx    #velocity in axis 
            #twist.linear.y = vy*1000   #steer angle
            twist.linear.y = vy   #steer angle
            #twist.angular.z = angular
            twist.linear.z = angular    #this is invalued
            self.velPublisher.publish(twist)
            # print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
            # print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
            # print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
            # rospy.loginfo("battery: {}".format(battery))
            # rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
            self.imuPublisher.publish(imu)
            self.magPublisher.publish(mag)
            self.volPublisher.publish(battery)
            self.EdiPublisher.publish(edition)
            
            sleep(1.0/self.rate_hz)  # Sleep for the specified rate
            #turn to radis
            steer_radis = vy*1000.0*3.1416/180.0
            state.position = [0, 0, steer_radis, 0, steer_radis, 0]
            if not vx == angular == 0:
                i = random.uniform(-3.14, 3.14)
                state.position = [i, i, steer_radis, i, steer_radis, i]
            self.staPublisher.publish(state)

    def RGBLightcallback(self, msg):
        # 流水灯控制，服务端回调函数 RGBLight control
        '''
        effect=[0, 6]，0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示
        speed=[1, 10]，数值越小速度变化越快。
        '''
        if not isinstance(msg, Int32): return
        # print ("RGBLight: ", msg.data)
        for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        # 蜂鸣器控制  Buzzer control
        if not isinstance(msg, Bool): return
        # print ("Buzzer: ", msg.data)
        if msg.data:
            for i in range(3): self.car.set_beep(1)
        else:
            for i in range(3): self.car.set_beep(0)

    def cmd_vel_callback(self, msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
        if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
        vx = msg.linear.x
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
        vy = msg.linear.y
        angular = msg.angular.z     # wait for change
        # 小车运动控制,vel: ±1, angular: ±5
        # Trolley motion control,vel=[-1, 1], angular=[-5, 5]
        # rospy.loginfo("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(vx, vy, angular))
        '''if self.nav_use_rotvel == True:
            #in navigation, we use angular info to instead steer rot
            vy = angular/1000.0/3.1416*180.0  #Radian system
            angular = 0.0
        else:
            vy = 0.0'''
        #self.car.set_car_motion(vx*0.8, vy, angular)
        self.car.set_car_motion(vx*0.7, vy, angular)
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)

    def dynamic_reconfigure_callback(self, config, level):
        # self.car.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
        # print("PID: ", config['Kp'], config['Ki'], config['Kd'])
        self.linear_max = config['linear_max']
        self.linear_min = config['linear_min']
        self.angular_max = config['angular_max']
        self.angular_min = config['angular_min']
        return config

def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = yahboomcar_driver()
        # Run pub_data in a separate thread to allow ROS2 spinning
        import threading
        pub_thread = threading.Thread(target=driver.pub_data, daemon=True)
        pub_thread.start()
        
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        driver.get_logger().error(f"Error: {str(e)}")
    finally:
        if 'driver' in locals():
            driver.cancel()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
