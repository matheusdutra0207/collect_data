#!/usr/bin/env python3
import pandas as pd

import rospy
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped

from is_wire.core import Channel, Subscription
from is_msgs.common_pb2 import Pose

def callback_ekf(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = data.pose.pose.orientation.z
    df_data['ekf']['x'].append(x)
    df_data['ekf']['y'].append(y)
    df_data['ekf']['yaw'].append(yaw)
    

def callback_amcl(data):
    x = data.pose.position.x
    y = data.pose.position.y
    yaw = data.pose.orientation.z
    df_data['amcl']['x'].append(x)
    df_data['amcl']['y'].append(y)
    df_data['amcl']['yaw'].append(yaw)

def data_gt_consume():
    message = channel_0.consume()
    pose_gt = message.unpack(Pose)
    x = pose_gt.position.x
    y = pose_gt.position.y
    yaw = pose_gt.orientation.yaw
    df_data['gt']['x'].append(x)
    df_data['gt']['y'].append(y)
    df_data['gt']['yaw'].append(yaw)

def data_reconstruction_consume():
    message = channel_1.consume()
    pose_reconstruction = message.unpack(Pose)
    x = pose_reconstruction.position.x
    y = pose_reconstruction.position.y
    yaw = pose_reconstruction.orientation.yaw
    df_data['reconstruction']['x'].append(x)
    df_data['reconstruction']['y'].append(y)
    df_data['reconstruction']['yaw'].append(yaw)

def save_data():
    df_data.to_csv(f'data{data_id}.csv', index=False)
    print("new data saved")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('pose/pose2', PoseWithCovarianceStamped, callback_ekf)
    rospy.Subscriber('pose3', PoseWithCovariance, callback_amcl)

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        data_gt_consume()
        data_reconstruction_consume()
        save_data()
        rate.sleep()

if __name__ == '__main__':
    data_id = 1

    position_data = {'gt':             {'x': [], 'y': [], 'yaw': []}, 
                    'reconstruction':  {'x': [], 'y': [], 'yaw': []},
                    'amcl':            {'x': [], 'y': [], 'yaw': []},
                    'ekf':             {'x': [], 'y': [], 'yaw': []}
                    }

    df_data = pd.DataFrame.from_dict(position_data)    

    channel_0 = Channel("amqp://guest:guest@localhost:5672")
    channel_1 = Channel("amqp://guest:guest@localhost:5672")

    subscription_0 = Subscription(channel_0)
    subscription_1 = Subscription(channel_1)

    subscription_0.subscribe(topic="pose.pose0")
    subscription_1.subscribe(topic="pose.pose1")
    listener()

