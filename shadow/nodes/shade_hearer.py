#!/usr/bin/env python
import rospy 
import roslib
roslib.load_manifest('shadow')
import math
import tf
import numpy as np
import numpy.linalg as la
import geometry_msgs.msg
import turtlesim.srv
import time
from std_msgs.msg import String
import PyKDL as KDL





def vector_cos(v1,v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2) + 0.0001
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2) + 0.0001
    return dot/(mag1*mag2)

def vector_angle(v1,v2):
    cos = vector_cos(v1,v2)
    return math.acos(cos)*180/math.pi

    

def vision_proc(listener):
    try:
        (rsh,qrsh) = listener.lookupTransform('/right_shoulder', '/openni_depth_frame', rospy.Time(0))
        (re,qre) = listener.lookupTransform('/right_elbow', '/openni_depth_frame', rospy.Time(0))
        (rh,qrh) = listener.lookupTransform('/right_hand', '/openni_depth_frame', rospy.Time(0))
        (lsh,qlsh) = listener.lookupTransform('/left_shoulder', '/openni_depth_frame', rospy.Time(0))
        (le,qle) = listener.lookupTransform('/left_elbow', '/openni_depth_frame', rospy.Time(0))
        (lh,qlh) = listener.lookupTransform('/left_hand', '/openni_depth_frame', rospy.Time(0))
        (ne,qne) = listener.lookupTransform('/neck', '/openni_depth_frame', rospy.Time(0))


        # rsh  = np.array(rsh)
        # re = np.array(re)
        # rh = np.array(rh)
        # lsh = np.array(lsh)
        # le = np.array(le)
        # lh = np.array(lh)
        # ne = np.array(ne)
        #
        # r_shld_blade = rsh - ne
        # l_shld_blade = lsh - ne
        # r_shld_2_elb = re - rsh
        # r_elb_2_hd = rh - re
        # l_shld_2_elb = le - lsh
        # l_elb_2_hd = lh - le
        # y_axis = np.array([0,1,0])
        # z_axis = np.array([0,0,1])

        # right_shld_angle = vector_angle(r_shld_blade,r_shld_2_elb)
        # right_elb_angle = vector_angle(r_shld_2_elb,r_elb_2_hd)
        #
        # print "Right Shoulder Angle :",
        # print right_shld_angle
        # print "Right Elbow Angle :",
        # print right_elb_angle

        re = KDL.Vector(re[0],re[1],re[2])
        rsh = KDL.Vector(rsh[0], rsh[1], rsh[2])
        rh = KDL.Vector(rh[0], rh[1], rh[2])
        r_shld_2_elb = re -rsh
        r_shld_2_elb.Normalize()
        r_elb_2_hd = rh - re
        r_elb_2_hd.Normalize()


        angle = -(math.asin(r_shld_2_elb.y()) + math.pi/2.0)
        print "Lift",
        print math.degrees(angle)

        angle = -(math.asin(r_elb_2_hd.y()) + math.pi / 2.0)
        print "Pan",
        print math.degrees(angle)





        #msg = map(str,list(lsh+le+lh+rsh+re+rh))
        msg = 'n'

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
