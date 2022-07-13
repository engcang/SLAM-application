#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import sys
import signal

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
import tf


def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class converter():
    def __init__(self):
        rospy.init_node('pcl_frame_changer', anonymous=True)
        self.sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/camera/depth/color/points/changed', PointCloud2, queue_size=10)
        self.pathsub = rospy.Subscriber('/path', Path, self.callback2)
        self.pathpub = rospy.Publisher('/path2', Path, queue_size=10)

        self.path = Path()
        self.path.header.frame_id = "camera_init"
        self.br = tf.TransformBroadcaster()
        self.counter = 0

    def callback2(self,data):
        lastpose = data.poses[len(data.poses)-1]
        lastpose.header.stamp = rospy.Time.now()
        lastpose.header.frame_id = "camera_init"
        self.path.poses.append(lastpose)
        self.path.header.stamp = rospy.Time.now()
        self.pathpub.publish(self.path)
        

    def callback(self,data):
        self.counter = self.counter+1
        self.br.sendTransform((0, 0, 0), (0.5,-0.5,0.5,-0.5), rospy.Time.now(), "camera_init", "map")
        if self.counter%7==0:
            new_data = PointCloud2()
            new_data = data
            new_data.header.frame_id = "aft_mapped"
            new_data.header.stamp = rospy.Time.now()
            self.pub.publish(new_data)


if __name__=='__main__':
    cvt=converter()
    time.sleep(1)
    while 1:
        pass
