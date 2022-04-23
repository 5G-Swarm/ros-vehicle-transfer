#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import ctypes
libgcc_s = ctypes.CDLL('/lib/x86_64-linux-gnu/libgcc_s.so.1')

import cv2
import numpy as np
import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from autoware_msgs.msg import TrackingObjectMarker, TrackingObjectMarkerArray
from sensor_msgs.msg import Image

import time
from informer import Informer
from proto.python_out import marker_msgs_pb2, geometry_msgs_pb2, path_msgs_pb2, cmd_msgs_pb2, ctrl_msgs_pb2

def parse_path(message):
    global path_pub
    print('recv path')
    path = path_msgs_pb2.Path()
    path.ParseFromString(message)

    ros_pose_array = Path()
    ros_pose_array.header.frame_id = "map"
    ros_pose_array.header.stamp = rospy.Time.now()
    for pose in path.poses:
        ros_pose = PoseStamped()
        ros_pose.pose.position.x = pose.x
        ros_pose.pose.position.y = pose.y
        ros_pose.pose.position.z = pose.theta
        ros_pose_array.poses.append(ros_pose)

    path_pub.publish(ros_pose_array)

def parse_ctrl(message):
    global ctrl_pub
    ctrl = ctrl_msgs_pb2.Ctrl()
    ctrl.ParseFromString(message)

    ros_ctrl = Vector3()
    ros_ctrl.x = ctrl.flag
    ros_ctrl.y = ctrl.v
    ros_ctrl.z = ctrl.w
    print('publish ctrl:', ros_ctrl)
    ctrl_pub.publish(ros_ctrl)

class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')
    
    def send_odm(self, message):
        self.send(message, 'odm')

    def send_cmd(self, message):
        self.send(message, 'cmd')

    def send_img(self, message):
        self.send(message, 'img')
    
    def path_recv(self):
        # try:
        self.recv('path', parse_path)
        # except:
            # print('recv path timeout !')

    def ctrl_recv(self):
        if True:#try:
            self.recv('ctrl', parse_ctrl)
        # except:
            # print('recv ctrl timeout !')

def ros_marker2pb(ros_marker : TrackingObjectMarker):
    marker = marker_msgs_pb2.Marker()
    # type: TrackingObjectMarker
    marker.time_stamp = ros_marker.header.stamp.secs
    marker.id = ros_marker.track_id
    marker.pose.position.x = ros_marker.marker.pose.position.x
    marker.pose.position.y = ros_marker.marker.pose.position.y
    marker.pose.position.z = ros_marker.marker.pose.position.z
    marker.pose.orientation.x = ros_marker.marker.pose.orientation.x
    marker.pose.orientation.y = ros_marker.marker.pose.orientation.y
    marker.pose.orientation.z = ros_marker.marker.pose.orientation.z
    marker.pose.orientation.w = ros_marker.marker.pose.orientation.w
    marker.scale.x = ros_marker.marker.scale.x
    marker.scale.y = ros_marker.marker.scale.y
    marker.scale.z = ros_marker.marker.scale.z
    marker.color.r = ros_marker.marker.color.r
    marker.color.g = ros_marker.marker.color.g
    marker.color.b = ros_marker.marker.color.b
    return marker

def parse_ros_marker_list(ros_marker_array: TrackingObjectMarkerArray):
    marker_list = marker_msgs_pb2.MarkerList()
    for ros_mark in ros_marker_array.markers:
        mark = ros_marker2pb(ros_mark)
        marker_list.marker_list.append(mark)
    return marker_list

def callback_mark_array(ros_marker_array: TrackingObjectMarkerArray):
    global ifm
    marker_list = parse_ros_marker_list(ros_marker_array)
    sent_data = marker_list.SerializeToString()
    # print('send', len(sent_data))
    ifm.send_msg(sent_data)


def ros_odometry2pb(odometry):
    pose = geometry_msgs_pb2.Pose()
    pose.position.x = odometry.pose.pose.position.x
    pose.position.y = odometry.pose.pose.position.y
    pose.position.z = odometry.pose.pose.position.z
    pose.orientation.x = odometry.pose.pose.orientation.x
    pose.orientation.y = odometry.pose.pose.orientation.y
    pose.orientation.z = odometry.pose.pose.orientation.z
    pose.orientation.w = odometry.pose.pose.orientation.w
    return pose

def ros_cmd2pb(ros_cmd):
    cmd = cmd_msgs_pb2.Cmd()
    cmd.v = ros_cmd.linear.x
    cmd.w = ros_cmd.angular.z
    return cmd

def callback_odometry(odometry):
    global ifm
    pose = ros_odometry2pb(odometry)
    sent_data = pose.SerializeToString()
    # print('send', len(sent_data))
    ifm.send_odm(sent_data)

def callback_cmd(ros_cmd):
    global ifm
    cmd = ros_cmd2pb(ros_cmd)
    sent_data = cmd.SerializeToString()
    ifm.send_cmd(sent_data)

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
    rospy.init_node('vehicle_5g_transfer', anonymous=True)
    ifm = Client(config = 'config.yaml')
    path_pub = rospy.Publisher('/global_path', Path, queue_size=0)
    ctrl_pub = rospy.Publisher('/manual_ctrl', Vector3, queue_size=0)
    rospy.Subscriber('/detection/lidar_detector/objects_markers_withID', TrackingObjectMarkerArray, callback_mark_array)
    rospy.Subscriber('/base2odometry', Odometry, callback_odometry)
    rospy.Subscriber('/camera/color/image_raw', Image, callback_img)
    rospy.Subscriber('/cmd_vel', Twist, callback_cmd)

    # rospy.spin()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
