"""
SpotMicroAI Simulation
"""

import pybullet_data
import time
import pybullet as p
import math
import numpy as np
from kinematics import Kinematic
from enum import Enum

class RobotState(Enum):
    OFF = 0     # don't do anything
    READY = 1   # compact, legs together, waiting
    STAND = 2   # standing, feet to the ground
    TROTTING_GAIT=3 # legs alway moving up/down 0/3,1/2 / 2 Step
    CRAWL = 4   # 4 Stepped, 1,2,3,0
    CRAWL2 = 5  #4 Stepped, Back first, 2,1,3,0

class Robot:

    def __init__(self,useFixedBase=False,useStairs=True,resetFunc=None):

        # Simulation Configuration
        self.useMaximalCoordinates = False
        self.resetFunc=resetFunc
        self.useRealTime = True
        self.debugLidar=False

        self.camera_angle = 0
        self.pauseCamera=False
        self.rotateCamera=False
        self.trackCamera=True

        self.debug=False
        self.fixedGains = False
        self.fixedTimeStep = 1. / 550
        self.numSolverIterations = 100 #200
        self.useFixedBase =useFixedBase
        self.useStairs=useStairs
        self.useBlocks = True

        self.init_oritentation=p.getQuaternionFromEuler([0, 0, 0]) # [0, 0, 90.0]
        self.init_position=[0, 0, 0]  # [-0.3, 0, 0.1]

        self.reflection=False
        self.state=RobotState.OFF
        # Parameters for Servos - still wrong
        self.kp = 1 #0.045#0.012
        self.kd = 1 # .4#.2
        self.maxForce = 4 #12.5

        self.physId = p.connect(p.SHARED_MEMORY)
        if (self.physId < 0):
            p.connect(p.GUI)
        self.angle = 90

        self.oldTextId=0
        self.textId=0
        self.oldDebugInfo=[]
        self.rot=(0,0,0)
        self.pos=(0,0,0)
        self.t=0
        if self.reflection:
            p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_reflection, 1)
        #p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
        self.IDkp = p.addUserDebugParameter("Kp", 0, 0.05, self.kp) # 0.05
        self.IDkd = p.addUserDebugParameter("Kd", 0, 1, self.kd) # 0.5
        self.IDmaxForce = p.addUserDebugParameter("MaxForce", 0, 50, self.maxForce)

        p.setRealTimeSimulation(self.useRealTime)

        self.quadruped = self.loadModels()
        self.createEnv()
        self.changeDynamics(self.quadruped)
        self.jointNameToId = self.getJointNames(self.quadruped)

        self.L=215
        self.W=75+5+40

        self.dirs = [[-1, 1, 1], [1, 1, 1], [-1, 1, 1], [1, 1, 1]]
        self.roll=0

        # Foot position array.
        # TODO(dwind): Understand why x-coords diff from front to back.
        self.Lp = np.array([
            [107.5, -100, self.W/2, 1],   #120  # front right toe?
            [107.5, -100, -self.W/2, 1],  #120  # front left toe?
            [-22, -100, self.W/2, 1],   #-50    # back right toe?
            [-22, -100, -self.W/2, 1]]) #-50    # back left toe?


        self.kin = Kinematic()
        p.setPhysicsEngineParameter(numSolverIterations=self.numSolverIterations)
        #p.setTimeStep(self.fixedTimeStep)

        p.setRealTimeSimulation(self.useRealTime)
        self.ref_time = time.time()
        p.getCameraImage(320,200)#160,100)
        p.resetDebugVisualizerCamera(1,85.6,0,[-0.61,0.12,0.25])
        # Camera Settings
        fov, aspect, nearplane, farplane = 90, 1.3, .0111, 100
        self.projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

        self.lastLidarTime=0

        self.override_angles = None
        self.last_cat_angles_degrees = [0] * 16
        self.catServo2dogServo = [
            (0, 0), (0, 0), (0, 0), (0, 0),  # hPan, hTilt, tPan, N/A
            (0, 0), (1, 0), (3, 0), (2, 0),  # FLR, FRR, HRR, HLR
            (0, 1), (1, 1), (3, 1), (2, 1),  # FLS, FRS, HRS, HLS
            (0, 2), (1, 2), (3, 2), (2, 2)]  # FLK, FRK, HRK, HLK
        self.catDir2dogDir = [
            1, 1, 1, 1,
            1, 1, 1, 1,
            -1, -1, 1, 1,
            -1, -1, 1, 1]
        self.catOffset2dogOffset = [
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            90, 90, 90, 90]

    def createEnv(self):
        if not self.useBlocks:
            return
        shift = [0, -0.0, 0]
        meshScale = [0.1, 0.1, 0.1]
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                            rgbaColor=[1, 1, 1, 1],
                                            specularColor=[0.4, .4, 0],
                                            halfExtents=[1,1,.5],
                                            visualFramePosition=shift)
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                            collisionFramePosition=shift,
                                            halfExtents=[1,1,.5])
        atexUid = p.loadTexture("concrete2.png")
        rangex = 5
        rangey = 5
        for i in range(rangex):
            for j in range(rangey):
                s=p.createMultiBody(baseMass=1000,
                                baseInertialFramePosition=[0, 0, 0],
                                baseCollisionShapeIndex=collisionShapeId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=[((-rangex / 2) + i) * 5,
                                                (-rangey / 2 + j) * 5, 1],
                                useMaximalCoordinates=True)
                p.changeVisualShape(s, -1, textureUniqueId=atexUid)


    def loadModels(self):
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -9.81)

        orn = p.getQuaternionFromEuler([math.pi/30*0, 0*math.pi/50, 0])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0], orn)
        p.changeDynamics(planeUid, -1, lateralFriction=100) # dwind: friction had a huge effect >2.
        texUid = p.loadTexture("concrete.png")
        p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)
        if self.useStairs:
            stairsUid = p.loadURDF("../urdf/stairs_gen.urdf.xml", [0, -1, 0], orn)
        flags=p.URDF_USE_SELF_COLLISION
        quadruped = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", self.init_position,
                            self.init_oritentation,
                            useFixedBase=self.useFixedBase,
                            useMaximalCoordinates=self.useMaximalCoordinates,
                            flags=flags) #p.URDF_USE_IMPLICIT_CYLINDER)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.changeDynamics(quadruped, -1, lateralFriction=1)  # E.g. https://github.com/OpenQuadruped/spot_mini_mini/blob/spot/spotmicro/spot.py

        return quadruped

    def changeDynamics(self,quadruped):
        nJoints = p.getNumJoints(quadruped)
        for i in range(nJoints):
            p.changeDynamics(quadruped, i, localInertiaDiagonal=[
                            0.000001, 0.000001, 0.000001])

    def getJointNames(self,quadruped):
        nJoints = p.getNumJoints(quadruped)
        jointNameToId = {}

        for i in range(nJoints):
            jointInfo = p.getJointInfo(quadruped, i)
            name = jointInfo[1].decode('UTF-8')
            jointNameToId[name] = jointInfo[0]
            print('Joint %d is named %s.' % (jointInfo[0], name))
        return jointNameToId


    def addInfoText(self,bodyPos): #bodyEuler,linearVel,angularVel
        # TODO: use replacementId instead of deleting the old views. seems to have memleak issues?
        if not self.debug:
            return
        #text="Distance: {:.1f}m".format(math.sqrt(bodyPos[0]**2+bodyPos[1]**2))
        #text2="Roll/Pitch: {:.1f} / {:.1f}".format(math.degrees(bodyEuler[0]),math.degrees(bodyEuler[1]))
        #text3="Vl: {:.1f} / {:.1f} / {:.1f} Va: {:.1f} / {:.1f} / {:.1f}".format(linearVel[0],linearVel[1],linearVel[2],
        #    angularVel[0],angularVel[1],angularVel[2])


        x,y=bodyPos[0],bodyPos[1]
        newDebugInfo=[
        p.addUserDebugLine([x, y, 0], [x, y, 1], [0,1,0]),
        #p.addUserDebugText(text, [x+0.03, y, 0.7], textColorRGB=[1, 1, 1], textSize=1.0),
        #p.addUserDebugText(text2, [x+0.03, y, 0.6], textColorRGB=[1, 1, 1], textSize=1.0),
        #p.addUserDebugText(text3, [x+0.03, y, 0.5], textColorRGB=[1, 1, 1], textSize=1.0),
        p.addUserDebugLine([-0.3, 0, 0], [0.3, 0, 0], [0,1,0], parentObjectUniqueId=self.quadruped, parentLinkIndex=1 ),
        p.addUserDebugLine([0, -0.2, 0], [0, 0.2, 0], [0,1,0], parentObjectUniqueId=self.quadruped, parentLinkIndex=1 )]

        if not self.override_angles:
            angleInfo = self.getAngleInfo()
            z = 0.5
            zStep = -0.05
            for leg in ['FL', 'BL', 'FR', 'BR']:
              text = '%s:  S:%7.1f  T:%7.1f  K:%7.1f' % (leg, angleInfo[leg]['S'], angleInfo[leg]['T'], angleInfo[leg]['K'])
              newDebugInfo.append(p.addUserDebugText(text, [x+0.03, y, z], textColorRGB=[1, 1, 1], textSize=1.0))
              z = z + zStep

        if len(self.oldDebugInfo)>0:
            for x in self.oldDebugInfo:
                p.removeUserDebugItem(x)
        self.oldDebugInfo=newDebugInfo


    def handleCamera(self,cubePos, cubeOrn):

        # Look forward/up
        init_camera_vector = (-1, 0, 0)
        init_up_vector = (0, 0, 1)

        # rotate camera on y by 25degrees
        orn = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, -math.pi/180*25,0]))

        # Rotate vectors with body
        orn = np.array(orn).reshape(3, 3)
        rot_matrix = p.getMatrixFromQuaternion(cubeOrn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        camera_vector = rot_matrix.dot(orn.dot(init_camera_vector))
        up_vector = rot_matrix.dot(orn.dot(init_up_vector))

        # Camera origin is right at the end of the head watching forward (Body-Coordinates)
        view_matrix = p.computeViewMatrix(
            cubePos + 0.17 * camera_vector, cubePos + 3 * camera_vector,  up_vector)
        img = p.getCameraImage(320, 200, view_matrix, self.projection_matrix)
        #TODO: Use the camera image

    def resetBody(self):
        self.ref_time=time.time()
        if len(self.oldDebugInfo)>0:
            for x in self.oldDebugInfo:
                p.removeUserDebugItem(x)
        p.resetBasePositionAndOrientation(self.quadruped, self.init_position,[0,0,0,1])
        p.resetBaseVelocity(self.quadruped, [0, 0, 0], [0, 0, 0])
        if(self.resetFunc):
            self.resetFunc()

    def checkSimulationReset(self,bodyOrn):
        (xr, yr, _) = p.getEulerFromQuaternion(bodyOrn)
        # If our Body rotated more than pi/3: reset
        if(abs(xr) > math.pi/3 or abs(yr) > math.pi/3):
            self.resetBody()
            return True
        return False

    def bodyRotation(self,rot):
        self.rot=rot

    def bodyPosition(self,pos):
        self.pos=pos

    def feetPosition(self,Lp):
        self.Lp=Lp

    def getPos(self):
        bodyPos,_=p.getBasePositionAndOrientation(self.quadruped)
        return bodyPos

    def getIMU(self):
        _, bodyOrn = p.getBasePositionAndOrientation(self.quadruped)
        linearVel, angularVel = p.getBaseVelocity(self.quadruped)
        return bodyOrn,linearVel,angularVel

    def getAngleInfo(self):
      if self.override_angles is not None:
          return self.override_angles
      angles = self.kin.calcIK(self.Lp, self.rot, self.pos)
      output = {}
      for lx, leg in enumerate(['FL', 'FR', 'BL', 'BR']):
          output[leg] = {}
          for px, part in enumerate(['S', 'T', 'K']):
              output[leg][part] = angles[lx][px]*self.dirs[lx][px]*180.0/math.pi
      return output

    def getLastCatAnglesDegrees(self, num_angles):
        offset = 16 - num_angles
        if offset < 4:
            offset = 4  # skip head & tail motors
        return self.last_cat_angles_degrees[offset:16]

    def setCatAnglesDegrees(self, angles):
        self.last_cat_angles_degrees = [0] * 16
        self.override_angles = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        offset = 16 - len(angles)
        if offset < 4:
            offset = 4  # skip head & tail motors
        for i in range(offset, 16):
            self.last_cat_angles_degrees[i] = angles[i-offset]
            (leg, part) = self.catServo2dogServo[i]
            angle = angles[i-offset] * self.catDir2dogDir[i] + self.catOffset2dogOffset[i]
            self.override_angles[leg][part] = angle * math.pi / 180.0

    def getCameraAngle(self):
        return self.camera_angle

    def setCameraAngle(self, angle):
        self.camera_angle = angle

    def fixForceAndGains(self):
        kp=p.readUserDebugParameter(self.IDkp)
        kd=p.readUserDebugParameter(self.IDkd)
        maxForce=p.readUserDebugParameter(self.IDmaxForce)
        self.kp = kp
        self.kd = kd
        self.maxForce = maxForce
        self.fixedGains = True
        print('Using fixed maxForce & Gains.')

    def toggleCameraPause(self):
        self.pauseCamera = not self.pauseCamera

    def step(self):

        if (self.useRealTime):
            self.t = time.time() - self.ref_time
        else:
            self.t = self.t + self.fixedTimeStep

        quadruped=self.quadruped
        bodyPos, bodyOrn = p.getBasePositionAndOrientation(quadruped)
        #linearVel, angularVel = p.getBaseVelocity(quadruped)
        #bodyEuler=p.getEulerFromQuaternion(bodyOrn)
        kp = self.kp
        kd = self.kd
        maxForce = self.maxForce
        if not self.fixedGains:
            kp=p.readUserDebugParameter(self.IDkp)
            kd=p.readUserDebugParameter(self.IDkd)
            maxForce=p.readUserDebugParameter(self.IDmaxForce)

        #self.handleCamera(bodyPos, bodyOrn)
        self.addInfoText(bodyPos)

        #if self.checkSimulationReset(bodyOrn):
        #    return False
        if not self.pauseCamera:
            if self.rotateCamera:
                p.resetDebugVisualizerCamera(0.7,self.t*10,-5,bodyPos)
            elif self.trackCamera:
                p.resetDebugVisualizerCamera(0.7, self.camera_angle, -5, bodyPos)
        # Calculate Angles with the input of FeetPos,BodyRotation and BodyPosition
        if self.override_angles:
            angles = self.override_angles
        else:
            angles = self.kin.calcIK(self.Lp, self.rot, self.pos)

        for lx, leg in enumerate(['front_left', 'front_right', 'rear_left', 'rear_right']):
            for px, part in enumerate(['shoulder', 'leg', 'foot']):
                j = self.jointNameToId[leg+"_"+part]
                p.setJointMotorControl2(bodyIndex=quadruped,
                                        jointIndex=j,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=angles[lx][px]*self.dirs[lx][px],
                                        positionGain=kp,
                                        velocityGain=kd,
                                        force=maxForce)


        if (self.useRealTime == False):
            p.stepSimulation()
            time.sleep(self.fixedTimeStep)
