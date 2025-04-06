import pybullet as p
import pybullet_data
import time
import math
import csv

# === INITIAL CONFIGURATION ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(0.005)

plane_id = p.loadURDF("plane.urdf")
robot_start_pos = [0, 0, 1]
robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF("robot/urdf/robot.urdf", robot_start_pos, robot_start_orientation)
cube_id = p.loadURDF("objects/cube.urdf", [0, 4, 0], robot_start_orientation)

wheel_joints = [7, 11, 15, 19]
arm_joints = {"q1": 21, "q2": 23, "q3": 27, "q4": 26}
gripper_joints = [29, 30, 31, 32]

def get_position_and_yaw(body_id):
    pos, orn = p.getBasePositionAndOrientation(body_id)
    yaw = p.getEulerFromQuaternion(orn)[2]
    return pos, yaw

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def drive(left_speed, right_speed):
    p.setJointMotorControl2(robot_id, wheel_joints[0], p.VELOCITY_CONTROL, targetVelocity=left_speed)
    p.setJointMotorControl2(robot_id, wheel_joints[1], p.VELOCITY_CONTROL, targetVelocity=left_speed)
    p.setJointMotorControl2(robot_id, wheel_joints[2], p.VELOCITY_CONTROL, targetVelocity=-right_speed)
    p.setJointMotorControl2(robot_id, wheel_joints[3], p.VELOCITY_CONTROL, targetVelocity=-right_speed)

def stop_wheels():
    for joint in wheel_joints:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=0)

def move_gripper(target_world_pos, speed=0.01, dt=0.005, threshold=0.075):
    gripper_index = 28
    distance = float("inf")

    # Iteratively move the gripper towards the target world position
    while distance > threshold:
        gripper_world = p.getLinkState(robot_id, gripper_index)[0]

        # Compute vector to target
        dx = target_world_pos[0] - gripper_world[0]
        dy = target_world_pos[1] - gripper_world[1]
        dz = target_world_pos[2] - gripper_world[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Compute small step in the direction of the target
        vx = speed * dx / distance
        vy = speed * dy / distance
        vz = speed * dz / distance

        new_target = (
            gripper_world[0] + vx,
            gripper_world[1] + vy,
            gripper_world[2] + vz
        )

        # Solve IK for the new intermediate position
        ik_result = p.calculateInverseKinematics(robot_id, gripper_index, new_target)

        # Extract joint angles (specific indices for this robot)
        q1 = ik_result[4]
        q2 = ik_result[5]
        q3 = ik_result[7]
        q4 = ik_result[6]

        # Apply joint positions to the robot arm
        p.setJointMotorControl2(robot_id, arm_joints["q1"], p.POSITION_CONTROL, targetPosition=q1, force=500, maxVelocity=5)
        p.setJointMotorControl2(robot_id, arm_joints["q2"], p.POSITION_CONTROL, targetPosition=q2, force=500, maxVelocity=5)
        p.setJointMotorControl2(robot_id, arm_joints["q3"], p.POSITION_CONTROL, targetPosition=q3, force=500, maxVelocity=5)
        p.setJointMotorControl2(robot_id, arm_joints["q4"], p.POSITION_CONTROL, targetPosition=q4, force=500, maxVelocity=5)

        # Advance the simulation
        p.stepSimulation()


def move_arm(target_q, steps=100, delay=0.005):
    # Get the current joint positions
    current_q = [
        p.getJointState(robot_id, arm_joints["q1"])[0],
        p.getJointState(robot_id, arm_joints["q2"])[0],
        p.getJointState(robot_id, arm_joints["q3"])[0],
        p.getJointState(robot_id, arm_joints["q4"])[0]
    ]

    # Interpolate joint movement towards the target configuration
    for i in range(steps):
        q_interp = [
            current_q[j] + (target_q[j] - current_q[j]) * (i + 1) / steps
            for j in range(4)
        ]

        # Apply interpolated joint positions
        p.setJointMotorControl2(robot_id, arm_joints["q1"], p.POSITION_CONTROL, targetPosition=q_interp[0], force=500, maxVelocity=5)
        p.setJointMotorControl2(robot_id, arm_joints["q2"], p.POSITION_CONTROL, targetPosition=q_interp[1], force=500, maxVelocity=5)
        p.setJointMotorControl2(robot_id, arm_joints["q3"], p.POSITION_CONTROL, targetPosition=q_interp[2], force=500, maxVelocity=5)
        p.setJointMotorControl2(robot_id, arm_joints["q4"], p.POSITION_CONTROL, targetPosition=q_interp[3], force=500, maxVelocity=5)

        # Advance the simulation with a delay for smooth motion
        p.stepSimulation()
        time.sleep(delay)


def operate_gripper(opening):
    p.setJointMotorControl2(robot_id, gripper_joints[0], p.POSITION_CONTROL, targetPosition=opening)
    p.setJointMotorControl2(robot_id, gripper_joints[1], p.POSITION_CONTROL, targetPosition=-opening)
    p.setJointMotorControl2(robot_id, gripper_joints[2], p.POSITION_CONTROL, targetPosition=opening)
    p.setJointMotorControl2(robot_id, gripper_joints[3], p.POSITION_CONTROL, targetPosition=-opening)

# === STATE MACHINE ===
state = "orient"
dt = 0.005
csv_filename = "Fase3_guillermo_alcocer.csv"
effort_data = []
effort_total = 0

while state != "end":
    robot_pos, robot_yaw = get_position_and_yaw(robot_id)
    cube_pos, _ = get_position_and_yaw(cube_id)

    if state == "orient":
        # Rotate the robot to face the cube
        dx = cube_pos[0] - robot_pos[0]
        dy = cube_pos[1] - robot_pos[1]
        desired_yaw = math.atan2(dy, dx)
        error = normalize_angle(abs(desired_yaw) - abs(robot_yaw))
        print(state)
        drive(3, -3)  # the wheels on one side turn in opposite direction

        if abs(error) < 0.6:
            stop_wheels()
            state = "move_forward"
            time.sleep(0.2)

    elif state == "move_forward":
        # Drive forward until close to the cube
        dx = cube_pos[0] - robot_pos[0]
        dy = cube_pos[1] - robot_pos[1]
        distance = math.hypot(dx, dy)
        print(state)

        if distance > 2.5:
            drive(4, 4)
        else:
            stop_wheels()
            state = "pick"
            time.sleep(0.2)

    elif state == "pick":
        # Perform pick-and-place sequence
        robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
        cube_above = (0, 4, 2)
        cube_grasp = (0, 4, 0.6)
        trailer_above = (robot_pos[0], robot_pos[1] - 0.5, 1.25)

        print("moving to cube")
        move_gripper(cube_above)

        print("lowering")
        move_gripper(cube_grasp)

        operate_gripper(0.06)  # Close gripper

        print("lifting")
        move_gripper(cube_above)

        move_arm([0, 0, 0, 1])  # Intermediate pose
        move_arm([1.884, 0.588, 0.066, 1.129])  # trailer container pose

        operate_gripper(0.0)  # Release object
        move_arm([0, 0, 0, 1])  # Return to rest position

        start_time = time.time()
        state = "move_after_pick"

    elif state == "move_after_pick":
        # Move forward for 3 seconds after placing the cube
        # to show the cube keep on trailer
        print(state)
        drive(4, 4)
        if time.time() - start_time >= 3.0:
            stop_wheels()
            state = "end"

    p.stepSimulation()
    time.sleep(dt)


    # === PHASE 3: Mechanical Effort Logging ===
    for joint_name, joint_idx in arm_joints.items():
        joint_state = p.getJointState(robot_id, joint_idx)
        torque = abs(joint_state[3])
        partial_effort = torque * dt
        effort_data.append([time.time(), joint_idx, partial_effort])
        effort_total += partial_effort

# === CSV OUTPUT ===
with open(csv_filename, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "JointIndex", "PartialEffort"])
    writer.writerows(effort_data)

print(f"Total Mechanical Effort = {round(effort_total, 4)}")
print("Process finished.")
