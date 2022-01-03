"""
Simulation of SpotMicroAI and it's Kinematics, using OpenCat Gait
"""
import pybullet as p
import numpy as np
import pybullet_data
import time
import math
import datetime as dt
import matplotlib.animation as animation
import random
from inputs import devices, get_gamepad
#from thinputs import ThreadedInputs
import spotmicroai
from kinematicMotion import KinematicMotion,TrottingGait
#from environment import environment
import catGaits

from threaded_keyboard import ThreadedKeyboard

rtime=time.time()
#env=environment()
def reset():
    global rtime
    rtime=time.time()

kb = ThreadedKeyboard()
kb.start()

robot=spotmicroai.Robot(False,False,reset)

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132
IDspurWidth = p.addUserDebugParameter("spur width", 0, robot.W, spurWidth)
IDstepHeight = p.addUserDebugParameter("step height", 0, 150, stepHeight)

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128


#IDheight = p.addUserDebugParameter("height", -40, 90, 40)

Lp = np.array([[iXf, -100,spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

resetPose()

catGait = catGaits.CatWalk() #CatWalk CatLY CatBD CatBK CatCR CatTR

# time.sleep(5)   # helpful when recording the screen

while True:

    #bodyPos=robot.getPos()
    #bodyOrn,_,_=robot.getIMU()
    #xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
    #distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)
    #if distance>500:
    #    robot.resetBody()

    #ir=xr/(math.pi/180)
    d=time.time()-rtime
    #height = p.readUserDebugParameter(IDheight)

    # wait 3 seconds to start
    if d>3:
        robot.setCatAnglesDegrees(catGait.getAngles(d-3))
    else:
        robot.setCatAnglesDegrees(catGait.getAngles(0))
        #robot.feetPosition(Lp)

    #roll=0
    #robot.bodyRotation((roll,math.pi/180*((joy_x)-128)/3,-(1/256*joy_y-0.5)))
    #bodyX=50+yr*10
    #robot.bodyPosition((bodyX, 40+height, -ir))
    robot.step()
    time.sleep(catGait.sleepSecs())

    key_pressed = kb.read()
    #print(f'Read key={key_pressed}.')
    if key_pressed == 'a':
        catGait = catGaits.CatWalkLeft()
    elif key_pressed == 'b':
        catGait = catGaits.CatBound()
    elif key_pressed == 'c':
        catGait = catGaits.CatCrawl()
    elif key_pressed == 'd':
        catGait = catGaits.CatWalkRight()
    elif key_pressed == 'e':
        angle = robot.getCameraAngle()
        robot.setCameraAngle(angle - 10)
    elif key_pressed == 'f':
        robot.fixForceAndGains()
    elif key_pressed == 'h':
        catGait = catGaits.CatStretch()
    elif key_pressed == 'm':
        catGait = catGaits.CatMarch()
    elif key_pressed == 'n':
        catGait = catGaits.CatSleep()
    elif key_pressed == 'o':
        catGait = catGaits.CatOurSit()
    elif key_pressed == 'p':
        catGait = catGaits.CatPushUp()
    elif key_pressed == 'q':
        angle = robot.getCameraAngle()
        robot.setCameraAngle(angle + 10)
    elif key_pressed == 'r':
        robot.resetBody()
        catGait = catGaits.CatBalance()
    elif key_pressed == 's':
        catGait = catGaits.CatBalance()
    elif key_pressed == 't':
        catGait = catGaits.CatTrot()
    elif key_pressed == 'v':
        catGait = catGaits.CatSit()
    elif key_pressed == 'w':
        catGait = catGaits.CatWalk()
    elif key_pressed == 'x':
        catGait = catGaits.CatBack()
    elif key_pressed == 'z':
        catGait = catGaits.CatZero()
    elif key_pressed == '?':
        catGait = catGaits.DadTwist()
