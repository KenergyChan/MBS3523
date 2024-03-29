import pybullet as p
import pybullet_data as pd
import time
import math
from attrdict import AttrDict

#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

wheels = [1, 2, 3, 4]
# (2, b'front_left_wheel', 0, 7, 6, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_left_wheel_link')
# (3, b'front_right_wheel', 0, 8, 7, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_right_wheel_link')
# (4, b'rear_left_wheel', 0, 9, 8, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_left_wheel_link')
# (5, b'rear_right_wheel', 0, 10, 9, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_right_wheel_link')

wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [0.5, -0.5, 0.5, -0.5]
wheelDeltasFwd = [0.3, 0.3, 0.3, 0.3]

baseorn = p.getQuaternionFromEuler([3.1415, 0, 0.3])
baseorn1 = [0, 0, 0, 1]
baspos = [0,0,0]

nJoints = 6
kukaEndEffectorIndex = 6

ikSolver = 0

prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0

trailDuration = 10

def connection():
    client = p.connect(p.SHARED_MEMORY)
    if (client < 0):
        p.connect(p.GUI)

def configPhysicsEngine():
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)   
    
def loadObj():
    armStartPos = [0, 0, 0]
    armStartOrn = p.getQuaternionFromEuler([0, 0, math.pi / 2])
    p.loadURDF("plane.urdf", [0, 0, 0])
    carId = p.loadURDF("AGV6/urdf/AGV6.urdf", basePosition=[0, 0, 0])
    kukaId = p.loadURDF("arm/urdf/arm.urdf", armStartPos, armStartOrn)
    arm_joint_info(kukaId)
    #"kuka_iiwa/model_free_base.urdf"
    #cubeArr = []
    #cubeArr2 = []
    #cubeId = p.loadURDF('cube.urdf', basePosition=[0.5, 0.8, 1])
    #for i in range(10):
    #    cubeId = p.loadURDF('cube.urdf', basePosition=[1.7, -i, 2])
    #    cubeArr2.append(cubeId)
    return carId, kukaId


def resetJointInitState(kukaId):
    jointPositions = [1.852, -1.091, 2.312, -0.231, 1.455, 0.959]
    for jointIndex in range(p.getNumJoints(kukaId)):
        p.resetJointState(kukaId, jointIndex, jointPositions[jointIndex])
        
def resetJointNullSpace(kukaId):
    for i in range(6):
        p.resetJointState(kukaId, i, rp[i])
    

def chassis_control_from_keyboard(huskyId):
    keys = p.getKeyboardEvents()
    wheelVelocities = [0, 0, 0, 0]
    speed = 5.0
    for k in keys:
        if p.B3G_LEFT_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasTurn[i] * 1.2
        if p.B3G_RIGHT_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasTurn[i] * 1.2
        if p.B3G_UP_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasFwd[i] * 1.2
        if p.B3G_DOWN_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasFwd[i] * 1.2

    for i in range(len(wheels)):
        p.setJointMotorControl2(huskyId,
                                wheels[i],
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheelVelocities[i],
                                force=2500)

    
def get_left_front_wheel_state(huskyId):
    worldPos, worldOrn = p.getLinkState(huskyId, 2)[:2]
    return worldPos, worldOrn

def get_arm_endefftector_state_pos_and_orn(kukaId):
    worldPos, worldOrn = p.getLinkState(kukaId, 6)[:2]
    return worldPos, worldOrn

def get_chassis_num_joints(huskyId):
    cnJ = p.getNumJoints(huskyId)
    return cnJ

def get_arm_num_joints(kukaId):
    anJ = p.getNumJoints(kukaId)
    return anJ

def chassis_joint_info(huskyId):
    for i in range(p.getNumJoints(huskyId)):
        print("[INFO] joint ", p.getJointInfo(huskyId, i))
        
def arm_joint_info(kukaId):
    for i in range(p.getNumJoints(kukaId)):
        print("[INFO] joint ", p.getJointInfo(kukaId, i))
        
def set_p2p_param():
    postionControlGp = []
    postionControlGp.append(p.addUserDebugParameter('X', -1.0, 2.5, 0))
    postionControlGp.append(p.addUserDebugParameter('Y', -1.0, 2.5, 0))
    postionControlGp.append(p.addUserDebugParameter('Z', 1.0, 2.5, 0.5))
    postionControlGp.append(p.addUserDebugParameter('Row', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('Pitch', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('Yaw', -math.pi, math.pi, 0)) 
    return postionControlGp

def set_joint_param():
    postionControlGp = []
    postionControlGp.append(p.addUserDebugParameter('j1', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j2', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j3', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j4', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j5', -math.pi, math.pi, 0))
    postionControlGp.append(p.addUserDebugParameter('j6', -math.pi, math.pi, 0)) 
    #postionControlGp.append(p.addUserDebugParameter('j7', -math.pi, math.pi, 0))
    return postionControlGp

def configJointMotor(kukaID, param):
    p.setJointMotorControl2(kukaID, 0, p.POSITION_CONTROL, targetPosition=param[0])
    p.setJointMotorControl2(kukaID, 1, p.POSITION_CONTROL, targetPosition=param[1])
    p.setJointMotorControl2(kukaID, 2, p.POSITION_CONTROL, targetPosition=param[2])
    p.setJointMotorControl2(kukaID, 3, p.POSITION_CONTROL, targetPosition=param[3])
    p.setJointMotorControl2(kukaID, 4, p.POSITION_CONTROL, targetPosition=param[4])
    p.setJointMotorControl2(kukaID, 5, p.POSITION_CONTROL, targetPosition=param[5])
    #p.setJointMotorControl2(kukaID, 6, p.POSITION_CONTROL, targetPosition=param[6])


def accurateCalculateInverseKinematics(kukaId, endEffectorId, targetPos, threshold, maxIter):
    closeEnough = False
    iter = 0
    dist2 = 1e30
    while (not closeEnough and iter < maxIter):
        jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, targetPos, ll, ul, jr, rp, jd)
        for i in range(6):
            p.resetJointState(kukaId, i, jointPoses[i])
        ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
    print ("Num iter: " + str(iter) + "threshold: " + str(dist2))
    return jointPoses


def moveTo(kukaId, pos, euler, flag):
    orn = p.getQuaternionFromEuler(euler)
    if flag == False:
        jointPoses = p.calculateInverseKinematics(kukaId,
                                              kukaEndEffectorIndex,
                                              pos,
                                              orn,
                                              jointDamping=jd,
                                              solver=ikSolver,
                                              maxNumIterations=100,
                                              residualThreshold=.01)
    else:
        jointPoses = accurateCalculateInverseKinematics(kukaId, 
                                                        kukaEndEffectorIndex, 
                                                        pos, 
                                                        0.001, 
                                                        100)
    
    for i in range(nJoints):
        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=1000,
                                positionGain=0.04,
                                velocityGain=1)
    print("[INFO] move to succ")  


def menu_p2p_control(kukaId, Gp):
    paramList = []
    for i in range(6):
        paramList.append(p.readUserDebugParameter(Gp[i]))
    
    pos = [paramList[0], paramList[1], paramList[2]]
    ornAngle = [paramList[3], paramList[4], paramList[5]]
    
    # print("Pos: ", pos)
    # print("Orn: ", ornAngle)
    moveTo(kukaId, pos, ornAngle, False)


def muti_point_path_planning(kukaId, fromPos, toPos):
    global hasPrevPose
    global prevPose
    global prevPose1
    
    for f, t in zip(fromPos, toPos):
        moveTo(kukaId, f, t, False)
        print("f:  ", f)
        print("t: ", t)
        for i in range(5):
            ls = p.getLinkState(kukaId, kukaEndEffectorIndex)[:1]
            print("index: ", i)
            print("pos", ls)
            if (hasPrevPose):
                #p.addUserDebugLine(prevPose, f, [0, 1, 0], 1, trailDuration)
                p.addUserDebugLine(prevPose1, ls[0], [0, 0, 1], 1, trailDuration)
            prevPose = f
            prevPose1 = ls[0]
            hasPrevPose = 1
            time.sleep(0.2)
    time.sleep(0.5)
    
def menu_joint_control(kukaId, Gp):
    param = []
    for i in range(6):
        param.append(p.readUserDebugParameter(Gp[i]))
    configJointMotor(kukaId, param)
    
    worldPos, worldOrn = p.getLinkState(kukaId, kukaEndEffectorIndex)[:2]
    print(f"world Pos {worldPos} & world Orn {worldOrn}")
    print("\n\n=================================\n\n")
      
def initEnv():
    connection()
    configPhysicsEngine()
    
    carId, kukaId = loadObj()
    resetJointInitState(kukaId)
    
    print("[INFO] Number of Arm Joints: ", get_arm_num_joints(kukaId))
    print("[INFO] Number of Chassis Joints: ", get_chassis_num_joints(carId))
    
    # stack arm on top of chassis 
    cid = p.createConstraint(kukaId, -1, carId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.5, 0.5, 1])
    p.changeConstraint(cid, [0,0,1] , jointChildFrameOrientation= baseorn,maxForce=50)
    p2pL = set_p2p_param()
    jointL = set_joint_param()
    time.sleep(2)
    """ curX = -0.33
    curY = -0.77
    print(curX, curY)
    pos = [float(curX), float(curY), float(1.5)]
    euler = [0, 0, 0]
    time.sleep(2)
    moveTo(kukaId, pos, euler) """
    
    fromPos = [[-0.75, -0.75, 1.5], [1.5, 1.5, 1.5]]
    toPos = [[1.5, 1.5, 1.5], [1, 1, 1]]


    
    while True:
        chassis_control_from_keyboard(carId)
        resetJointInitState(kukaId)    
        time.sleep(0.5)
        """ wwp, wwo = getLeftFrontWheelState(huskyId)
        print("=======================================================")
        print("state x: ", wwp[0])
        print("state y: ", wwp[1])
        print("state end effector: ", get_arm_endefftector_state(kukaId))
        print("=======================================================")
        if wwp[1] > -1.0:
            pos = [float(wwp[0]), float(-1.0), float(1.5)]
            euler = [2.24, 1.6, 1.8]
            time.sleep(1)
            moveTo(kukaId, pos, euler)  """
        
        #pos = [float(0.8), float(0.8), float(1.5)]    
        #euler = [2.24, 1.6, 1.8]
        #moveTo(kukaId, pos, euler)
        #menu_p2p_control(kukaId, p2pL)
        #muti_point_path_planning(kukaId, fromPos, toPos)
        #menu_joint_control(kukaId, jointL)
        
        
if __name__=="__main__":
    initEnv()