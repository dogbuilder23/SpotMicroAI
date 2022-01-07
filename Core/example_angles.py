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
import spotmicroai
from kinematicMotion import KinematicMotion,TrottingGait
#from environment import environment

import catGaits
from threaded_keyboard import ThreadedKeyboard

rtime=time.time()

def reset():
    global rtime
    rtime=time.time()

# -------------------- main() ----------------

kb = ThreadedKeyboard()
kb.start()

robot=spotmicroai.Robot(False, False, reset)

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132
IDspurWidth = p.addUserDebugParameter("spur width", 0, robot.W, spurWidth)
IDstepHeight = p.addUserDebugParameter("step height", 0, 150, stepHeight)

Lp = np.array([
    [iXf, -100,  spurWidth, 1],
    [iXf, -100, -spurWidth, 1],
    [-50, -100,  spurWidth, 1],
    [-50, -100, -spurWidth, 1]])

elapsed_time = time.time() - rtime
catGait = catGaits.CatWalk(elapsed_time)

# time.sleep(5)   # helpful when recording the screen

initial_wait_secs = 3   # TODO(dwind): Why is this wait needed? Do we need to call robot.step() or just sleep?

while True:
    elapsed_time = time.time() - rtime

    catAngles = robot.getLastCatAnglesDegrees(catGait.getNumServos())
    # TODO(dwind): This will re-apply initial_wait_secs on every reset.
    if elapsed_time < initial_wait_secs: # wait 3 seconds to start
        catGait.adjustStartTime(elapsed_time)
    else:
        catAngles = catGait.getAngles(elapsed_time, catAngles)
    robot.setCatAnglesDegrees(catAngles)
    robot.step()

    time.sleep(0.001)  # TODO(dwind): Fix at 0.02? 0.1?

    key_pressed = kb.read()
    #print(f'Read key={key_pressed}.')
    if key_pressed == 'a':
        catGait = catGaits.CatWalkLeft(elapsed_time)
    elif key_pressed == 'b':
        catGait = catGaits.CatBound(elapsed_time)
    elif key_pressed == 'c':
        catGait = catGaits.CatCrawl(elapsed_time)
    elif key_pressed == 'd':
        catGait = catGaits.CatWalkRight(elapsed_time)
    elif key_pressed == 'e':
        angle = robot.getCameraAngle()
        robot.setCameraAngle(angle + 10)
    elif key_pressed == 'f':
        robot.fixForceAndGains()
    elif key_pressed == 'h':
        catGait = catGaits.CatStretch(elapsed_time)
    elif key_pressed == 'm':
        catGait = catGaits.CatMarch(elapsed_time)
    elif key_pressed == 'n':
        catGait = catGaits.CatSleep(elapsed_time)
    elif key_pressed == 'o':
        catGait = catGaits.CatOurSit(elapsed_time)
    elif key_pressed == 'p':
        catGait = catGaits.CatPushUp(elapsed_time)
    elif key_pressed == 'q':
        angle = robot.getCameraAngle()
        robot.setCameraAngle(angle - 10)
    elif key_pressed == 'r':
        robot.resetBody() # Sets rtime, see reset() above.
        elapsed_time = time.time() - rtime
        catGait = catGaits.CatBalance(elapsed_time)
    elif key_pressed == 's':
        catGait = catGaits.CatBalance(elapsed_time)
    elif key_pressed == 't':
        catGait = catGaits.CatTrot(elapsed_time)
    elif key_pressed == 'v':
        catGait = catGaits.CatSit(elapsed_time)
    elif key_pressed == 'w':
        catGait = catGaits.CatWalk(elapsed_time)
    elif key_pressed == 'x':
        catGait = catGaits.CatBack(elapsed_time)
    elif key_pressed == 'y':
        catGait = catGaits.DadYawLeft(elapsed_time)
    elif key_pressed == 'z':
        catGait = catGaits.CatZero(elapsed_time)
    elif key_pressed == '?':
        catGait = catGaits.DadTwist(elapsed_time)
    elif key_pressed == '[':
        catGait = catGaits.DadRollRight(elapsed_time)
    elif key_pressed == '-':
        catGait = catGaits.JustRollRight(elapsed_time)
    elif key_pressed == '\\':
        catGait = catGaits.TestPose(elapsed_time)
    elif key_pressed == '.':
        robot.toggleCameraPause()
