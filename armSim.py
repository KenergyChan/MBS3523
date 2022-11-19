import pybullet as p
import pybullet_data as pd
import math
import time
from collections import namedtuple
from attrdict import AttrDict


def configMotor(tankId, param):
    
    p.setJointMotorControl2(
        tankId,
        0,
        p.POSITION_CONTROL,
        targetPosition=param[0]
    )
    p.setJointMotorControl2(
        tankId,
        1,
        p.POSITION_CONTROL,
        targetPosition=param[1]
    )
    p.setJointMotorControl2(
        tankId,
        2,
        p.POSITION_CONTROL,
        targetPosition=param[2]
    )
    p.setJointMotorControl2(
        tankId,
        3,
        p.POSITION_CONTROL,
        targetPosition=param[3]
    )
    p.setJointMotorControl2(
        tankId,
        4,
        p.POSITION_CONTROL,
        targetPosition=param[4]
    )
    
    p.setJointMotorControl2(
        tankId,
        5,
        p.POSITION_CONTROL,
        targetPosition=param[5]
    )
    
def addParam():
    postionControlGp = []
    postionControlGp.append(p.addUserDebugParameter('j1', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j2', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j3', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j4', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j5', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j6', -math.pi, math.pi, 0))

    return postionControlGp

def menuControl(id, Gp):
    param = []
    for i in range(6):
        param.append(p.readUserDebugParameter(Gp[i]))
    
    configMotor(id, param)
        
def initialPose1(tankId, nj):
    #jointPoses = p.calculateInverseKinematics(tankId, 5, [0, -1, 0.5], [0, 0, 0])
    jointPoses = [0,0,0,math.pi/4,math.pi/2,0]
    #print("pos", jointPoses)
    for i in range(nj):
        # enable motor to move
        p.setJointMotorControl2(
            bodyIndex=tankId,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPoses[i],
            targetVelocity=0,
            force=500,
            positionGain=0.09,
            velocityGain=1,
        )
        
def moveTo1(tankId, nj):
    #jointPoses = p.calculateInverseKinematics(tankId, 5, pos, orn)
    jointPoses = [0, 0, 0, math.pi / 4, math.pi / 2, 0]
    #print("pos", jointPoses)
    for i in range(nj):
        # enable motor to move
        p.setJointMotorControl2(
            bodyIndex=tankId,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPoses[i],
            targetVelocity=0,
            force=500,
            positionGain=0.09,
            velocityGain=1,
        )

def moveTo2(tankId, nj):
    #jointPoses = p.calculateInverseKinematics(tankId, 5, pos, orn)
    jointPoses = [0, 0, 0, -math.pi / 4, math.pi / 2, 0]
    #print("pos", jointPoses)
    for i in range(nj):
        # enable motor to move
        p.setJointMotorControl2(
            bodyIndex=tankId,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPoses[i],
            targetVelocity=0,
            force=500,
            positionGain=0.09,
            velocityGain=1,
        )

def menuP2PArm1(tankId, nj):
    x = input('x: ')
    y = input('y: ')
    z = input('z: ')
    pos = [float(x), float(y), float(z)]
    orn = p.getQuaternionFromEuler([0, 0, math.pi])
    moveTo1(tankId, nj, pos, orn)
    
    
def routine(tankId, nJoints):
    # 0.5, 1.1, 0.6 0.5, -0.1, 0.6
    posSeq = [(0.63, 1.3, 0.3), (0.4, -0.1, 0.3), (0.63, 1.3, 0.3)]
    ornSeq = [p.getQuaternionFromEuler([0, 0, math.pi]), p.getQuaternionFromEuler([0, 0, math.pi]), p.getQuaternionFromEuler([0, 0, math.pi])]
    for i in range(3):
        

        print("=================================\n\n")
        #moveTo2(tankId, nJoints, posSeq[i], ornSeq[i])
        print("posSeq", posSeq[i])
        print("ornSeq", ornSeq[i])
        
        worldPos, worldOrn = p.getLinkState(tankId, 11)[:2]
        print(f"world Pos {worldPos} & world Orn {worldOrn}")
        print("\n\n=================================\n\n")
        time.sleep(3)
        
def grasp(tankId, nJoints):
    # 0.5, 1.1, 0.6 0.5, -0.1, 0.6
    posSeq = [(0, -1, 0.3), (1, 0, 0.3), (0, -1, 0.3)]
    ornSeq = [p.getQuaternionFromEuler([0, 0, math.pi]), p.getQuaternionFromEuler([0, 0, math.pi]), p.getQuaternionFromEuler([0, 0, math.pi])]
    print(len(posSeq))
    for i in range(3):
        print("=================================\n\n")
        moveTo1(tankId, nJoints, posSeq[i], ornSeq[i])
        print("posSeq", posSeq[i])
        print("ornSeq", ornSeq[i])
        
        worldPos, worldOrn = p.getLinkState(tankId, 1)[:2]
        print(f"world Pos {worldPos} & world Orn {worldOrn}")
        print("\n\n=================================\n\n")
        time.sleep(1)
    
def initArmEnv():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    move_up = 1
    move_down = 0
    # pre-build urdf
    planeId = p.loadURDF("plane.urdf")
    
    p.setGravity(0, 0, 0)
    p.setRealTimeSimulation(0)
    p.setPhysicsEngineParameter(enableConeFriction=0)
   
    armStartPos = [0, 0 ,0]
    armStartOrn = p.getQuaternionFromEuler([0, 0, math.pi/2])
    
    # please change to your own directory 
    tankId = p.loadURDF("arm/urdf/arm.urdf", armStartPos, armStartOrn,useFixedBase=1)
    #cubeId = p.loadURDF("cube.urdf", basePosition=[-2, -2, 0])
    
    nJoints = p.getNumJoints(tankId)
    jointInfo = namedtuple(
        "jointInfo",
        ["id", "name", "type", "lowerlimit", "upperlimit", "maxforce", "maxVelocity"]
    )
    
    joints = AttrDict()
    # get each joint info
    for i in range(nJoints):
        info = p.getJointInfo(tankId, i)
        jointId = info[0]
        jointName = info[1].decode("utf-8")
        jointTy = info[2]
        jointLow = info[8]
        jointHigh = info[9]
        jointMaxForce = info[10]
        jointMaxVelocity = info[11]       
        single = jointInfo(
            jointId,
            jointName,
            jointTy,
            jointLow,
            jointHigh,
            jointMaxForce,
            jointMaxVelocity
        )
        joints[single.name] = single
    
    print(joints)
    
    # add control param for 12 joints
    paramList = addParam()
    
    # calculate 6 joint angle given 3d point coord
    # initialPose(tankId, nJoints)
    bp, bo = p.getBasePositionAndOrientation(tankId)
    print("Base Pos & Orn: ", bp, bo)
    initialPose1(tankId, nJoints)
    time.sleep(0.1)
    while True:
        # menu control from user input
        p.stepSimulation()
        print("baby: ", p.getJointState(tankId, 3)[0],p.getJointState(tankId, 4)[0])
        if move_up==1:
            moveTo1(tankId, 6)
            time.sleep(0.0003)
        elif move_down==1:
            moveTo2(tankId, 6)
            time.sleep(0.0003)
        if p.getJointState(tankId,3)[0] > 0.78:
            move_up = 0
            move_down = 1
        if p.getJointState(tankId, 3)[0] < -0.78:
            move_up = 1
            move_down = 0

        #p.applyExternalTorque(tankId, 2, torqueObj=[0.,5e-4,0.], flags=p.LINK_FRAME, physicsClientId=PYB_CLIENT) # INCORRECT ?
        #paramList = [0,0,0,0,0,5]
        #menuControl(tankId, paramList)

        # p.getCameraImage(480, 320)
        #menuP2PArm1(tankId, nJoints)
            #grasp(tankId, 6)
        #time.sleep(1)
        #routine(tankId, 2)

        
        
        
if __name__=="__main__":
    initArmEnv()
