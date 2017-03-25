#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import time

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    print "right hand"
    #turtle_vel = rospy.Publisher('tf_head', geometry_msgs.,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,quaternion) = listener.lookupTransform('/right_elbow', '/right_shoulder', rospy.Time(0))
            (transl,quaternionl) = listener.lookupTransform('/left_elbow', '/left_shoulder', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        euler = tf.transformations.euler_from_quaternion(quaternion)
        eulerl = tf.transformations.euler_from_quaternion(quaternionl)
        
        euler = tuple([x*180/math.pi for x in euler])
        eulerl = tuple([x*180/math.pi for x in eulerl])
        
        sign = trans[1]/abs(trans[1])
      	signl = transl[1]/abs(transl[1])
        #print "x: "+str(trans[0]) + " y: " + str(trans[1])+ " z: "+ str(trans[2])+ "pitch: " +str(sign*90-euler[1])
        #print "Left Shoulder: "+str(sign * eulerl[1])+ " Right Shoulder: " +str(sign*euler[1])+"\n"
        print "trans: " + str(trans)+"\n"
        print " quat: " + str(euler)+"\n\n"
        time.sleep(1)