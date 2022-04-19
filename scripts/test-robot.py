#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, BatteryState, Imu
from mavros_msgs.msg import Altitude#, EstimatorStatus
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from informer import Informer

def parse_cmd(message):
    print("Get cmd:", message)

class Client(Informer):
    def send_img(self, message):
        self.send(message, 'img')
    
    def cmd_recv(self):
        self.recv('cmd', parse_cmd)

def callback_altitude(ros_altitude):
    _ = ros_altitude.amsl

def callback_battery(ros_battery):
    _ = ros_battery.cell_voltage
    _ = ros_battery.percentage

def callback_gps_vel(ros_gps_vel):
    _ = ros_gps_vel.twist.linear.x
    _ = ros_gps_vel.twist.linear.y
    _ = ros_gps_vel.twist.linear.z
    _ = ros_gps_vel.twist.angular.x
    _ = ros_gps_vel.twist.angular.y
    _ = ros_gps_vel.twist.angular.z
    
def callback_imu(ros_imu):
    _ = ros_imu.angular_velocity.x
    _ = ros_imu.angular_velocity.y
    _ = ros_imu.angular_velocity.z

    _ = ros_imu.orientation.x
    _ = ros_imu.orientation.y
    _ = ros_imu.orientation.z
    _ = ros_imu.orientation.w

    # _ = imu.linear_acceleration.x
    # _ = imu.linear_acceleration.y
    # _ = imu.linear_acceleration.z

def callback_odom(ros_odom):
    _ = ros_odom.pose.pose.orientation.w
    _ = ros_odom.pose.pose.orientation.x
    _ = ros_odom.pose.pose.orientation.y
    _ = ros_odom.pose.pose.orientation.z

    _ = ros_odom.pose.pose.position.x
    _ = ros_odom.pose.pose.position.y
    _ = ros_odom.pose.pose.position.z

    _ = ros_odom.twist.twist.angular.x
    _ = ros_odom.twist.twist.angular.y
    _ = ros_odom.twist.twist.angular.z

    _ = ros_odom.twist.twist.linear.x
    _ = ros_odom.twist.twist.linear.y
    _ = ros_odom.twist.twist.linear.z

def callback_img(ros_img):
    global ifm
    img = np.ndarray(shape=(480, 640, 3), dtype=np.dtype("uint8"), buffer=ros_img.data)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    _, jpeg = cv2.imencode('.jpg', img)
    data = jpeg.tobytes()
    ifm.send_img(data)
    # cv2.imshow('img', img)
    # cv2.waitKey(2)

if __name__ == '__main__':
    ifm = Client(config = 'config.yaml')

    rospy.init_node('drone_5g_transfer', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback_img)
    rospy.Subscriber('/mavros/altitude', Altitude, callback_altitude)
    rospy.Subscriber('/mavros/battery', BatteryState, callback_battery)
    rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, callback_gps_vel)
    rospy.Subscriber('/mavros/imu/data', Imu, callback_imu)
    rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)

    # rospy.spin()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
