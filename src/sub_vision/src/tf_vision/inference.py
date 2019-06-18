#!/usr/bin/env python
import rospy as ros
import geometry_msgs.msg as geo 

def main():
    pub = ros.Publisher('tf_perception', geo.Point, queue_size=10)
    ros.init_node('vision_inference_node')
    rate = ros.Rate(50)
    while not ros.is_shutdown:
        pub.publish(0, 0, 0)
        rate.sleep()

