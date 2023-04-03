#!/usr/bin/env python3
import pandas as pd
import sys

import rospy
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from is_msgs.camera_pb2 import FrameTransformations, FrameTransformation
from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Pose

def callback_ekf(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = data.pose.pose.orientation.z
    df_data['ekf']['x'].append(x)
    df_data['ekf']['y'].append(y)
    df_data['ekf']['yaw'].append(yaw)
    print("new pose ekf")
    

def callback_amcl(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = data.pose.pose.orientation.z
    df_data['amcl']['x'].append(x)
    df_data['amcl']['y'].append(y)
    df_data['amcl']['yaw'].append(yaw)
    print("new pose amcl")

def data_gt_consume():
    message = channel_0.consume()
    pose_gt = message.unpack(FrameTransformation)
    x = pose_gt.tf.doubles[0]
    y = pose_gt.tf.doubles[1]
    yaw = pose_gt.tf.doubles[2]
    df_data['gt']['x'].append(x)
    df_data['gt']['y'].append(y)
    df_data['gt']['yaw'].append(yaw)
    print("new pose gt")

def data_reconstruction_consume():
    message = channel_1.consume()
    pose_reconstruction = message.unpack(Pose)
    x = pose_reconstruction.position.x
    y = pose_reconstruction.position.y
    yaw = pose_reconstruction.orientation.yaw
    df_data['reconstruction']['x'].append(x)
    df_data['reconstruction']['y'].append(y)
    df_data['reconstruction']['yaw'].append(yaw)
    print("new pose reconstruction")

def save_data():
    df_data.to_csv(f'data{data_id}.csv', index=False)
    print("new data saved")

def listener():
    rospy.init_node('listener', anonymous=True)
    if 'ekf' in consume_list:
        rospy.Subscriber('/odometry/filtered_map', Odometry, callback_ekf)

    if 'amcl' in consume_list:
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback_amcl)

    print("sub in all topics")

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        if 'rec_rc' in consume_list:
            data_gt_consume()
        if 'rec' in consume_list:
            data_reconstruction_consume()
        save_data()
        rate.sleep()

if __name__ == '__main__':
    data_id = int(sys.argv[1])
    aruco_id = 5
    consume_list = sys.argv[2:]
    brocker_uri = "amqp://10.20.5.2:30000"

    input("Pressione ENTER para continuar")
    position_data = {}
    if 'amcl' in consume_list:
        position_data['amcl'] = {'x': [], 'y': [], 'yaw': []}
    if 'ekf' in consume_list:
        position_data['ekf'] = {'x': [], 'y': [], 'yaw': []}
    if 'rec' in consume_list:
        position_data['reconstruction'] = {'x': [], 'y': [], 'yaw': []}        
    if 'rec_rc' in consume_list:
        position_data['gt'] = {'x': [], 'y': [], 'yaw': []}       

    df_data = pd.DataFrame.from_dict(position_data)    

    channel_0 = Channel(brocker_uri)
    channel_1 = Channel(brocker_uri)

    subscription_0 = Subscription(channel_0)
    subscription_1 = Subscription(channel_1)

    subscription_0.subscribe(topic=f"localization.{aruco_id}.aruco")
    subscription_1.subscribe(topic=f"reconstruction.{aruco_id}.ArUco")
    listener()