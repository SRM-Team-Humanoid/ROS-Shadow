#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from skeleton_markers.msg import Skeleton
import PyKDL as KDL
import math
from math import acos, asin, pi



class SkellAngler(object):

    def __init__(self):
        rospy.init_node('skeleton_angles', anonymous=False)
        rospy.on_shutdown(self.end)

        self.HALF_PI = pi / 2.0

        rospy.Subscriber('skeleton', Skeleton, self.handler)
        self.pub = rospy.Publisher('skel_angles',String,queue_size=1)
        self.rate = rospy.Rate(10)
        self.msg= "n"

        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()

        while not rospy.is_shutdown():
            if self.msg != 'n':
                self.pub.publish(self.msg)

            self.rate.sleep()



    def handler(self, msg):
        for joint in msg.name:
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            self.skeleton['position'][joint] = KDL.Vector(msg.position[msg.name.index(joint)].x,
                                                          msg.position[msg.name.index(joint)].y,
                                                          msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[msg.name.index(joint)].x,
                                                                          msg.orientation[msg.name.index(joint)].y,
                                                                          msg.orientation[msg.name.index(joint)].z,
                                                                          msg.orientation[msg.name.index(joint)].w)


        #Right Arm

        right_shoulder_elbow = self.skeleton['position']['right_elbow'] - self.skeleton['position']['right_shoulder']
        right_arm_elbow_flex_hand = self.skeleton['position']['right_hand'] - self.skeleton['position']['right_elbow']
        right_shoulder_hand = self.skeleton['position']['right_hand'] - self.skeleton['position']['right_shoulder']

        right_shoulder_elbow.Normalize()
        right_shoulder_hand.Normalize()
        right_arm_elbow_flex_hand.Normalize()

        right_arm_shoulder_lift_angle = math.degrees(-(asin(right_shoulder_elbow.y()) + self.HALF_PI))
        right_arm_shoulder_pan_angle = math.degrees(-asin(right_arm_elbow_flex_hand.x()))
        right_arm_elbow_flex_angle = 0.66*math.degrees(acos(KDL.dot(right_shoulder_elbow, right_arm_elbow_flex_hand)))

        # Left Arm

        left_shoulder_elbow = self.skeleton['position']['left_elbow'] - self.skeleton['position']['left_shoulder']
        left_arm_elbow_flex_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_elbow']
        left_shoulder_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_shoulder']

        left_shoulder_elbow.Normalize()
        left_shoulder_hand.Normalize()
        left_arm_elbow_flex_hand.Normalize()

        left_arm_shoulder_lift_angle = math.degrees(asin(left_shoulder_elbow.y()) + self.HALF_PI)
        left_arm_shoulder_pan_angle = math.degrees(asin(left_arm_elbow_flex_hand.x()))
        left_arm_elbow_flex_angle = 0.66*math.degrees(-acos(KDL.dot(left_shoulder_elbow, left_arm_elbow_flex_hand)))

        right_femur = self.skeleton['position']['right_knee'] - self.skeleton['position']['right_hip']
        right_femur.Normalize()
        right_thigh_angle = math.degrees((asin(right_femur.y()) + self.HALF_PI))


        left_femur = self.skeleton['position']['left_knee'] - self.skeleton['position']['left_hip']
        left_femur.Normalize()
        left_thigh_angle = math.degrees((asin(left_femur.y()) + self.HALF_PI))

        waist = self.skeleton['position']['right_hip'] - self.skeleton['position']['left_hip']
        waist.Normalize()
        waist_angle = math.degrees((asin(waist.z())))


        # print waist_angle
        # print "Right Knee",
        # print right_thigh_angle
        # print "Left Knee",
        # print left_thigh_angle
        # print "Right Lift ",
        # print right_arm_shoulder_lift_angle
        # print "Right Pan ",
        # print right_arm_shoulder_pan_angle
        # print "Right Elbow ",
        # print right_arm_elbow_flex_angle
        #
        # print "Left Lift ",
        # print left_arm_shoulder_lift_angle
        # print "Left Pan ",
        # print left_arm_shoulder_pan_angle
        # print "Left Elbow ",
        # print left_arm_elbow_flex_angle


        self.msg = " ".join(map(str, [right_arm_shoulder_lift_angle, left_arm_shoulder_lift_angle, right_arm_shoulder_pan_angle,
                                      left_arm_shoulder_pan_angle,right_arm_elbow_flex_angle,left_arm_elbow_flex_angle,right_thigh_angle,
                                      left_thigh_angle,waist_angle]))







    def end(self):
        print "The End"



if __name__ == '__main__':
    try:
        SkellAngler()
    except rospy.ROSInterruptException:
        pass
