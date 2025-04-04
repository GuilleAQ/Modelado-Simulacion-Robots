import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI) # connect motor with gui
p.setRealTimeSimulation(1)#############################################################################################################################
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0, -9.8)

planeId = p.loadURDF("plane.urdf") #load model

startPos = [0, 0, 1]
euler_angles = [0,0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF("robot/urdf/robot.urdf",startPos, startOrientation) 

cube_id = p.loadURDF("objects/cube.urdf", [0, 0, 3.5], startOrientation)


numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

joints = [0, 1]
for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))
    print("Link - %s" % (p.getJointInfo(robotId,j)[12]))

# wheel_joints = [3, 7, 11, 15]
# # hand_cylinder_joint = 23
# q1_joint = 17
# q2_joint = 19
# q3_joint = 23
# q4_joint = 22 # ¿21?

# left_finger_joint = 25
# right_finger_joint = 26
# front_finger_joint = 27
# back_finger_joint = 28

wheel_joints = [4, 11, 15, 19]
# hand_cylinder_joint = 23
q1_joint = 21
q2_joint = 23
q3_joint = 27
q4_joint = 26 # ¿21?

left_finger_joint = 29
right_finger_joint = 30
front_finger_joint = 31
back_finger_joint = 32

wheel_links_vel = p.addUserDebugParameter(" wheel_links_vel", -10, 10, 0)

q1_pose = p.addUserDebugParameter(" q1_pose", -3.14, 3.14, 0)
q2_pose = p.addUserDebugParameter(" q2_pose", -4.34, 1.2, 0)
q3_pose = p.addUserDebugParameter(" q3_pose", -3.14, 3.14, 0)
q4_pose = p.addUserDebugParameter(" q4_pose", -1.2, 1.3, 0)

finger_slider = p.addUserDebugParameter(" finger_open_close", 0, 0.06, 0)

for j in range(16, 27):
    info = p.getJointInfo(robotId, j)
    print(f"Joint {j} - {info[1].decode('utf-8')}")
    print(f"  Type: {info[2]} (0=REVOLUTE, 1=PRISMATIC, 4=FIXED)")
    print(f"  Limits: {info[8]} to {info[9]}")
    print(f"  Max force: {info[10]}, Max velocity: {info[11]}")
    print()


input("Press enter to start motion")
while True:
    # p.setJointMotorControlArray(robotId, joints, p.VELOCITY_CONTROL, targetVelocities=[11] * 2)

    tl_vel = p.readUserDebugParameter(wheel_links_vel)
    q1p = p.readUserDebugParameter(q1_pose)
    q2p = p.readUserDebugParameter(q2_pose)
    q3p = p.readUserDebugParameter(q3_pose)
    q4p = p.readUserDebugParameter(q4_pose)
    finger_pos = p.readUserDebugParameter(finger_slider)

    p.setJointMotorControl2(robotId, wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=tl_vel)
    p.setJointMotorControl2(robotId, wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=tl_vel)
    p.setJointMotorControl2(robotId, wheel_joints[2], p.VELOCITY_CONTROL, targetVelocity=-tl_vel)
    p.setJointMotorControl2(robotId, wheel_joints[3], p.VELOCITY_CONTROL, targetVelocity=-tl_vel)
    
    p.setJointMotorControl2(robotId, q1_joint, p.POSITION_CONTROL, targetPosition=q1p)
    p.setJointMotorControl2(robotId, q2_joint, p.POSITION_CONTROL, targetPosition=q2p)
    p.setJointMotorControl2(robotId, q3_joint, p.POSITION_CONTROL, targetPosition=q3p)
    p.setJointMotorControl2(robotId, q4_joint, p.POSITION_CONTROL, targetPosition=q4p)
    p.setJointMotorControl2(robotId, left_finger_joint, p.POSITION_CONTROL, targetPosition=finger_pos)
    p.setJointMotorControl2(robotId, right_finger_joint, p.POSITION_CONTROL, targetPosition=-finger_pos)
    p.setJointMotorControl2(robotId, front_finger_joint, p.POSITION_CONTROL, targetPosition=finger_pos)
    p.setJointMotorControl2(robotId, back_finger_joint, p.POSITION_CONTROL, targetPosition=-finger_pos)


    # p.stepSimulation()
    time.sleep(1./240.)
