#!/usr/bin/env python
import rospy 
import roslib
roslib.load_manifest('shadow')
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import time
from std_msgs.msg import String

def vision_proc(listener):

    try:
        (rsh,_) = listener.lookupTransform('/right_shoulder', '/openni_depth_frame', rospy.Time(0))
        (re,_) = listener.lookupTransform('/right_elbow', '/openni_depth_frame', rospy.Time(0))
        (rh,_) = listener.lookupTransform('/right_hand', '/openni_depth_frame', rospy.Time(0))
        (lsh,_) = listener.lookupTransform('/left_shoulder', '/openni_depth_frame', rospy.Time(0))
        (le,_) = listener.lookupTransform('/left_elbow', '/openni_depth_frame', rospy.Time(0))
        (lh,_) = listener.lookupTransform('/left_hand', '/openni_depth_frame', rospy.Time(0))
	(ne,_) = listener.lookupTransform('/neck', '/openni_depth_frame', rospy.Time(0))
        msg = map(str,list(ne+lsh+le+lh+rsh+re+rh))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        msg = ['n']

    
    return " ".join(msg)

def vision_pub():
    pub = rospy.Publisher('vision_data', String, queue_size=10)
    rospy.init_node('Vision', anonymous=True)
    rate = rospy.Rate(10) # 10hz	    
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
		msg = vision_proc(listener)
		if msg != 'n':
			pub.publish(msg)
		rate.sleep()
		
if __name__ == '__main__':
    try:
    	vision_pub()
    except rospy.ROSInterruptException:
        pass
