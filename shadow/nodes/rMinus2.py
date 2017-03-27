import random
import pypot.dynamixel
import time
import numpy as np
from pprint import pprint
import xml.etree.cElementTree as ET
from collections import Counter
from copy import deepcopy
#from readchar import readchar
class Dxl(object):
    def __init__(self,port_id=0, scan_limit=25, lock=-1):
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
tree = XmlTree('data.xml')
tree2 = XmlTree('soccer.xml')
walk = Action(tree.superparsexml("22 F_S_L",offsets=[darwin]))
balance = MotionSet(tree.parsexml("152 Balance"), offsets=[darwin,hand])
moon_walk = Action(tree2.superparsexml("11 B_L_S", offsets=[darwin]))
lback = MotionSet(tree2.parsexml("18 B_L_E"), offsets=[darwin])
rback = MotionSet(tree2.parsexml("17 B_R_E"), offsets=[darwin])

bls1 = MotionSet(tree2.parsexml("14 B_L_S"),speed=2, offsets=[darwin])
bls2 = MotionSet(tree2.parsexml("16 B_L_M"),speed=2.1, offsets=[darwin])
bls3 = MotionSet(tree2.parsexml("15 B_R_M"),speed=2.1, offsets=[darwin])

bl1 = MotionSet(tree.parsexml("52 B_E_L"), offsets=[darwin])
bl2 = MotionSet(tree.parsexml("53 "), offsets=[darwin])

br1 = MotionSet(tree.parsexml("54 B_E_R"), offsets=[darwin])
br2 = MotionSet(tree.parsexml("55 "), offsets=[darwin])


l_step = MotionSet(tree2.parsexml("10 ff_l_r"), speed=1.5, offsets=[darwin])
r_step = MotionSet(tree2.parsexml("9 ff_r_l"), speed=1.5, offsets=[darwin])
l_attack = MotionSet(tree.parsexml("21 L attack"),speed=1.2,offsets=[darwin])
kick = MotionSet(tree.parsexml("18 L kick"),speed=2,offsets=[darwin])
f_getup = MotionSet(tree.parsexml("27 F getup"),speed=2.7,offsets=[darwin])
b_getup = MotionSet(tree.parsexml("28 B getup  "),speed=1.5,offsets=[darwin])
r_inv = MotionSet(tree2.parsexml("19 RFT"),speed=1.2,offsets=[darwin])
l_inv = MotionSet(tree2.parsexml("20 LFT"),speed=1.2,offsets=[darwin])
r_turn = MotionSet(tree2.parsexml("27 RT"),speed=1.2,offsets=[darwin])
l_turn = MotionSet(tree2.parsexml("28 LT"),speed=1.2,offsets=[darwin])
w1 = MotionSet(tree.parsexml("32 F_S_L"),speed=2.1,offsets=[darwin])
w2 = MotionSet(tree.parsexml("33 "),speed=2.1,offsets=[darwin])
w3 = MotionSet(tree.parsexml("38 F_M_R"),speed=2.7,offsets=[darwin])
w4 = MotionSet(tree.parsexml("39 "),speed=2.1,offsets=[darwin])
w5 = MotionSet(tree.parsexml("36 F_M_L"),speed=2.7,offsets=[darwin])
w6 = MotionSet(tree.parsexml("37 "),speed=2.1,offsets=[darwin])

b1 = MotionSet(tree.parsexml("46 B_S_R"),speed=2.1,offsets=[darwin])
b2 = MotionSet(tree.parsexml("47 "),speed=2.1,offsets=[darwin])
b3 = MotionSet(tree.parsexml("48 B_M_L"),speed=2.1,offsets=[darwin])
b4 = MotionSet(tree.parsexml("49 "),speed=2.1,offsets=[darwin])
b5 = MotionSet(tree.parsexml("50 B_M_R"),speed=2.1,offsets=[darwin])
b6 = MotionSet(tree.parsexml("51 "),speed=2.1,offsets=[darwin])

boom_walk = Action([l_step,r_step])
bwalk_init = Action([bls1])
bwalk_motion = Action([bls2,bls3])

bwalk = Action([bl1,bl2,br1,br2])

walk_init = Action([w1,w2])
walk_motion = Action([w3,w4,w5,w6])
#--------------------------------------------------------------------------------------------------------------#


if __name__=='__main__':
    dxl = Dxl(lock=20)
    state = dxl.getPos()
    print state
    raw_input("Proceed?")
    balance.execute()
    raw_input("Sure?")
    #boom_walk.execute(iter=5,speed=1.1)
    bwalk_init.execute(iter=1,speed=1.1)
    bwalk_motion.execute(iter=10,speed=1.1)
    #bwalk.execute(iter=5,speed=1.1)
