import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

robotId = p.loadURDF("urdf/ejercicio1.urdf", startPosition, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))

body_joint = 0
top_joint = 1
# body_link_vel = p.addUserDebugParameter("body_link_vel", -0.5, 0.5, 0)
body_link_pos = p.addUserDebugParameter(" body_link_pos", -1, 1, 0)

top_link_vel = p.addUserDebugParameter(" top_link_vel", -1, 1, 0)

for i in range(10000):
    # bl_vel = p.readUserDebugParameter(body_link_vel)
    bl_pos = p.readUserDebugParameter(body_link_pos)
    # p.setJointMotorControl2(robotId, body_joint, p.VELOCITY_CONTROL, targetVelocity=bl_vel)
    p.setJointMotorControl2(robotId, body_joint, p.POSITION_CONTROL, targetPosition=bl_pos)
    

    tl_vel = p.readUserDebugParameter(top_link_vel)
    p.setJointMotorControl2(robotId, top_joint, p.VELOCITY_CONTROL, targetVelocity=tl_vel)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
