#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import PointCloud2

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class lidar_repair():
    def __init__(self):
        rospy.init_node('lidar_repair', anonymous=True)
        self.pcl_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.vcallback)
        self.pcl_publisher = rospy.Publisher('/velody',PointCloud2, queue_size=1)
        self.rate = rospy.Rate(30)

    def vcallback(self, msg):
        w = msg.width
        h = msg.height
        msg.width = h
        msg.height = w
        self.pcl_publisher.publish(msg)
	    	    
if __name__=='__main__':
    ldr = lidar_repair()
    time.sleep(1)
    while True:
        try:
            ldr.rate.sleep()	    
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
