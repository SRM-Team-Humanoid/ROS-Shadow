#!/usr/bin/env python
import random
import pypot.dynamixel
import time
import numpy as np
from pprint import pprint
import xml.etree.cElementTree as ET
from collections import Counter
from copy import deepcopy
from readchar import readchar
import rospy
from std_msgs.msg import String

path = "/home/warr/cdatkin_ws/src/shadow/nodes/"

class Dxl(object):
    def __init__(self,port_id=0, scan_limit=25, lock=-1,debug=False):
        # Initializes Dynamixel Object
        # Port ID is zero by default
        ports = pypot.dynamixel.get_available_ports()
        if not ports:
            raise IOError('no port found!')

        print('ports found', ports)
        print('connecting on the first available port:', ports[port_id])
        dxl_io = pypot.dynamixel.DxlIO(ports[port_id])
        ids = dxl_io.scan(range(25))
        print(ids)

        if lock > 0:
            if len(ids) < lock:
                raise RuntimeError("Couldn't detect all motors.")

        self.dxl_io = dxl_io
        self.ids = ids
        if debug:
            speeds = [120 for x in ids]
            self.dxl_io.set_moving_speed(dict(zip(ids,speeds)))
    def setPos(self,pose):
        '''
        for k in pose.keys():
            if k not in self.ids:
                del pose[k]
        '''
        writ = {key: value for key, value in pose.items() if key in self.ids}
        #print writ
        self.dxl_io.set_goal_position(writ)

    def getPos(self):
        return Motion(1," ".join(map(str,self.dxl_io.get_present_position(self.ids))),0)



class XmlTree(object):

    def __init__(self,str):
        try:
            with open(str) as f:
                pass
            self.tree = ET.ElementTree(file=str)
        except:
            raise RuntimeError("File not found.")

    def parsexml(self,text):
        find = "PageRoot/Page[@name='" + text + "']/steps/step"
        motions = []
        prev_frame = 0
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            print find
            raise RuntimeError("ParseFail!")
        for step in steps:
            motion = Motion(step.attrib['frame'], step.attrib['pose'], prev_frame)
            prev_frame = step.attrib['frame']
            motions.append(motion)

        return motions

    def superparsexml(self,text,exclude=[],offsets=[]):
        find = "FlowRoot/Flow[@name='"+text+"']/units/unit"
        steps = [x for x in self.tree.findall(find)]
        if len(steps)==0:
            print find
            raise RuntimeError("ParseFail!")
        motionsets = []
        for step in steps:
            motionsets.append(MotionSet(self.parsexml(step.attrib['main']),speed=float(step.attrib['mainSpeed']),exclude=exclude,offsets=[]))

        return motionsets



class Motion(object):
    def __init__(self,frame,pose,prev_frame):
        self.frame = int(frame)
        self.pose = {}
        self.delay = self.frame-int(prev_frame)
        for i,p in enumerate(pose.split()):
            self.pose[i+1] =float(p)

    def __str__(self):
        return "Frame:"+str(self.frame) + "      Delay:"+str(self.delay) + "     Pose:"+" ".join(map(str,self.pose.values()))

    def updatePose(self,offset,add=True):
        if add:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] += offset[k]
        else:
            for k in offset.keys():
                if offset[k]=='i':
                    self.pose[k]=-self.pose[k]
                else:
                    self.pose[k] -= offset[k]


    def write(self,state, speed,exclude=[],offset={}):
        begpos = state.pose
        endpos = self.pose
        frames = []
        ids = []
        for k in endpos.keys():
            try:
                begpos[k]
            except:
                begpos[k]=0
            if begpos[k]!=endpos[k] and k not in exclude:
                frames.append(np.linspace(begpos[k],endpos[k],self.delay))
                ids.append(k)

        frames = zip(*frames)
        for f in frames:
            writ = dict(zip(ids, f))
            dxl.setPos(writ)
            time.sleep(0.008 / speed)
            print writ



class MotionSet(object):
    def __init__(self,motions,speed=1.0,exclude =[],offsets=[]):
        self.motions = motions
        self.speed = speed
        self.exclude = exclude
        self.offsets = offsets
        self.loaded = False

    def setExclude(self,list):
        self.exclude = list

    def setSpeed(self,speed):
        self.speed = speed

    def execute(self,speed=-1,iter=1):
        global state
        if speed<0:
            speed = self.speed
        if not self.loaded:
            for offset in self.offsets:
                for motion in self.motions:
                    motion.updatePose(offset)
            self.loaded = True

        while iter>0:
            for motion in self.motions:
                motion.write(state,speed,self.exclude)
                state = deepcopy(motion)
            iter-=1

class Action():
    def __init__(self,motionsets):
        self.motionsets=motionsets

    def add(self,motionsets):
        self.motionsets.extend(motionsets)

    def execute(self,iter=1,speed=1):
        while iter>0:
            for motionset in self.motionsets:
                for m in motionset.motions:
                    print m
                orig = motionset.speed
                motionset.speed = motionset.speed*speed
                motionset.execute()
                motionset.speed = orig
            iter -= 1


#--------------------------------------------------------------------------------------------------------------#
darwin = {1: 90, 2: -90, 3: 67.5, 4: -67.5, 7: 45, 8: -45, 9: 'i', 10: 'i', 13: 'i', 14: 'i', 17: 'i', 18: 'i'}
abmath = {11: 15, 12: -15, 13: -10, 14: 10, 15: -5, 16: 5}
hand = {5: 60, 6: -60}
tree = XmlTree(path+'data.xml')
tree2 = XmlTree(path+'soccer.xml')
walk = Action(tree.superparsexml("22 F_S_L",offsets=[darwin]))
balance = MotionSet(tree.parsexml("152 Balance"), offsets=[darwin,hand])
moon_walk = Action(tree2.superparsexml("11 B_L_S", offsets=[darwin]))
lback = MotionSet(tree2.parsexml("18 B_L_E"), offsets=[darwin])
rback = MotionSet(tree2.parsexml("17 B_R_E"), offsets=[darwin])
r_step = MotionSet(tree2.parsexml("10 ff_l_r"), speed=1.5,exclude=[1,2,3,4,5,6], offsets=[darwin])
l_step = MotionSet(tree2.parsexml("9 ff_r_l"), speed=1.5,exclude=[1,2,3,4,5,6], offsets=[darwin])
l_attack = MotionSet(tree.parsexml("21 L attack"),speed=1.2,offsets=[darwin])
kick = MotionSet(tree.parsexml("18 L kick"),speed=2,exclude=[1,2,3,4,5,6],offsets=[darwin])
f_getup = MotionSet(tree.parsexml("27 F getup"),speed=2.7,offsets=[darwin])
b_getup = MotionSet(tree.parsexml("28 B getup  "),speed=1.5,offsets=[darwin])
r_inv = MotionSet(tree2.parsexml("19 RFT"),speed=1.2,offsets=[darwin])
l_inv = MotionSet(tree2.parsexml("20 LFT"),speed=1.2,offsets=[darwin])
r_turn = MotionSet(tree2.parsexml("27 RT"),speed=1.2,exclude=[1,2,3,4,5,6],offsets=[darwin])
l_turn = MotionSet(tree2.parsexml("28 LT"),speed=1.2,exclude=[1,2,3,4,5,6],offsets=[darwin])
w1 = MotionSet(tree.parsexml("32 F_S_L"),speed=2.1,offsets=[darwin])
w2 = MotionSet(tree.parsexml("33 "),speed=2.1,offsets=[darwin])
w3 = MotionSet(tree.parsexml("38 F_M_R"),speed=2.7,offsets=[darwin])
w4 = MotionSet(tree.parsexml("39 "),speed=2.1,offsets=[darwin])
w5 = MotionSet(tree.parsexml("36 F_M_L"),speed=2.7,offsets=[darwin])
w6 = MotionSet(tree.parsexml("37 "),speed=2.1,offsets=[darwin])
boom_walk = Action([l_step,r_step])
walk_init = Action([w1,w2])
walk_motion = Action([w3,w4,w5,w6])
#--------------------------------------------------------------------------------------------------------------#



#
# prev_step = ""
# def getAngles(data):
#     global prev_step
#     angles = data.data
#     switch = angles.split()[0]
#     if switch == 'left_turn':
#         print 'l_turn'
#         prev_step = ""
#         print 'l_turn done'
#         #l_turn.execute()
#     elif switch == 'right_turn':
#         print 'r_turn'
#         prev_step = ""
#         print 'r_turn done'
#         #r_turn.execute()
#     elif switch == 'left_step' and prev_step!='left_step':
#         print 'l_step'
#         prev_step = 'left_step'
#         #l_step.execute()
#         print 'l_step done'
#     elif switch =='right_step' and prev_step!='right_step':
#         print 'r_step'
#         prev_step = 'right_step'
#         #r_step.execute()
#         print 'r_step done'
#     elif switch=='shad':
#         print "Do Shadow"
#         angles = " ".join(angles.split()[1:])
#         angles = Motion(10,angles,0)
#         print angles.pose
#         conversion = {1:'i',2:'i',3:'i'}
#         angles = MotionSet([angles],speed=1,offsets=[conversion])
#         #angles.execute()
#
#

def getAngles(data):
    global prev_step
    angles = data.data
    switch = angles.split()[0]
    if switch == 'left_turn':
        print 'l_turn'
        # l_turn.execute()
    elif switch == 'right_turn':
        print 'r_turn'
        # r_turn.execute()
    elif switch == 'left_step':
        print 'l_step'
        # l_step.execute()
    elif switch =='right_step':
        print 'r_step'
        # r_step.execute()
    elif switch=='shad':
        print "S"
        # print "Do Shadow"
        # angles = " ".join(angles.split()[1:])
        # angles = Motion(10,angles,0)
        # print angles.pose
        # conversion = {1:'i',2:'i',3:'i'}
        # angles = MotionSet([angles],speed=1,offsets=[conversion])
        # angles.execute()


prev_step = "None"
def shadow(data):
    global prev_step
    angles = data.data
    angles = " ".join(angles.split()[:-3])
    angles = Motion(10,angles,0)
    conversion = {1:'i',2:'i',3:'i'}
    angles = MotionSet([angles],speed=1,offsets=[conversion])
    rka,lka,wa = map(float,data.data.split()[-3:])

    if wa < -45:
        print 'Left Turn'
        prev_step = "None"
        l_turn.execute()
    elif wa > 45:
        print 'Right Turn'
        prev_step = "None"
        r_turn.execute()
    elif rka > 25 and prev_step!='r_step':
        print 'Right Step'
        prev_step = "r_step"
        r_step.execute()
    elif lka > 25 and prev_step!='l_step':
        print 'Left Step'
        prev_step = "l_step"
        l_step.execute()
    else:
        print "."
        angles.execute()



if __name__=='__main__':
    dxl = Dxl(lock=20,debug=True)
    state = dxl.getPos()
    print state
    raw_input("Proceed?")
    balance.execute()
    raw_input("Sure?")



    rospy.init_node('shadow_bot', anonymous=True)
    rospy.Subscriber('skel_angles',String,shadow,queue_size=1)
    rospy.spin()

    
