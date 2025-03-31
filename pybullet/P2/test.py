import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI) # connect motor with gui
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0, -9.8)

planeId = p.loadURDF("plane.urdf") #load model

startPos = [0, 0, 1]
euler_angles = [0,0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF("robot/urdf/robot.urdf",startPos, startOrientation) 

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

joints = [0, 1]
for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))
    print("Link - %s" % (p.getJointInfo(robotId,j)[12]))


input("Press enter to start motion")
while True:
    p.setJointMotorControlArray(robotId, joints, p.VELOCITY_CONTROL, targetVelocities=[11] * 2)

