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

wheel_joints = [3, 7, 11, 15]
wheel_links_vel = p.addUserDebugParameter("wheel_links_vel", -10, 10, 0)

input("Press enter to start motion")
while True:
    # p.setJointMotorControlArray(robotId, joints, p.VELOCITY_CONTROL, targetVelocities=[11] * 2)

    tl_vel = p.readUserDebugParameter(wheel_links_vel)
    p.setJointMotorControl2(robotId, wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=tl_vel)
    p.setJointMotorControl2(robotId, wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=tl_vel)
    p.setJointMotorControl2(robotId, wheel_joints[2], p.VELOCITY_CONTROL, targetVelocity=-tl_vel)
    p.setJointMotorControl2(robotId, wheel_joints[3], p.VELOCITY_CONTROL, targetVelocity=-tl_vel)
    # p.setJointMotorControlArray(robotId, wheel_joints, p.VELOCITY_CONTROL, targetVelocities=[tl_vel] * 4)
    # p.setJointMotorControl2(robotId, wheel_joints, p.VELOCITY_CONTROL, targetVelocity=tl_vel)

    # p.stepSimulation()
    time.sleep(1./240.)
