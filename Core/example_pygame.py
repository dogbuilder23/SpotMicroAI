"""
Simulation of SpotMicroAI and it's Kinematics 
Use a Pygame joystick to see how it works
"""
import pybullet as p
import numpy as np
import pybullet_data
import time
import math
import pygame
from pygame import locals
import spotmicroai


def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

def scaleValue(val):
    return (val + 1) * 128

def handleGamepad():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz
    events = pygame.event.get()
    for e in events:
        #print('Processing event.joy=%s' % e)
        if e.joy != 0:
            return
        command = 'UNKNOWN'
        if e.type == pygame.locals.JOYAXISMOTION: # 7
            value = scaleValue(e.value)
            print('Axis=%d value=%5.1f' % (e.axis, value))
            if e.axis == 0:
                joy_z = scaleValue(-1 * e.value)
            elif e.axis == 1:
                joy_y = scaleValue(-1 * e.value)
            elif e.axis == 2:
                joy_x = scaleValue(e.value)
            elif e.axis == 3:
                joy_rz = scaleValue(e.value)

robot=spotmicroai.Robot()
IDheight = p.addUserDebugParameter("height", -40, 90, 0)
IDroll = p.addUserDebugParameter("roll", -20, 20, 0)
#robot.feetPosition(Lp)
resetPose()

pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        handleGamepad()
    
        height = p.readUserDebugParameter(IDheight)
        roll = p.readUserDebugParameter(IDroll)
    
        # map the Gamepad Inputs to Pose-Values. Still very hardcoded ranges. 
        # TODO: Make ranges depend on height or smth to keep them valid all the time
        robot.bodyRotation((math.pi/180*roll,1/256*joy_x-0.5,-(0.9/256*joy_y-0.45)))
        robot.bodyPosition((100/256*-joy_rz-20+120, 40+height, 60/256*joy_z-30))
        robot.step()
