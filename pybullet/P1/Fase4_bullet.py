import pybullet as p
import pybullet_data
import time
import numpy as np
import csv


# --- Configuración de la cámara ---
camera_target_position = [3, 9.5, 4]
camera_distance = 12
camera_yaw = 90
camera_pitch = -35

# --- Configuración de posiciones y orientaciones ---
euler_angles = [0, 0, 3.1415/2]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPositionRobot = [0, 0, 1]
startPositionRamp = [0, 10, 0.75]
startPositionBarrier = [-1.25, 17, 0.5]
startPositionGoal = [0, 20, 0.1]

# --- Configuración de las ruedas ---
wheels_joints = [2 , 3, 4, 5]
wheels_joints_back = [4, 5]
target_velocity = 10
wheel_velocity = 11
wheel_force = 25

# --- Parámetros de control ---
umbral_ori = 0.06
umbral_vel = 2
max_wheel_velocity = 16
desired_velocity = 2.0  # Velocidad objetivo en m/s

# --- Configuración de fricción ---
lateralFriction = 0.93
spinningFriction = 0.005
rollingFriction = 0.003

# --- Configuración de la barrera ---
m, d, h, w = 5.0, 0.2, 0.2, 2.0  # Parámetros de masa y dimensiones

# --- Inicialización del almacenamiento de datos ---
data = []
last_y = 0  # Para verificar avance de 0.01 metros
start_time = time.time()
dt = 0.01

# --- posición meta del husky ---
goal_pose_y = startPositionGoal[1]


# --- Inicialización de la simulación en PyBullet ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                             cameraYaw=camera_yaw,
                             cameraPitch=camera_pitch,
                             cameraTargetPosition=camera_target_position)

# --- Carga de los modelos en la simulación ---
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("husky/husky.urdf", startPositionRobot, startOrientation)
rampId = p.loadURDF("urdf/ramp.urdf", startPositionRamp, startOrientation)
barrierId = p.loadURDF("urdf/barrier.urdf", startPositionBarrier, startOrientation)
goalId = p.loadURDF("urdf/goal.urdf", startPositionGoal, startOrientation)



p.setRealTimeSimulation(1)


# --- Configuración de las ruedas y la fricción ---
for wheel_joint in wheels_joints:
    # print(f"Joint número: {wheel_joint}")
    # fase 3.1
    p.setJointMotorControl2(robotId, wheel_joint, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity, force=wheel_force)

    # fase 3.2
    p.changeDynamics(robotId, wheel_joint, lateralFriction=lateralFriction, 
                     spinningFriction=spinningFriction, rollingFriction=rollingFriction)



# --- Cálculo de los momentos de inercia de la barrera ---
I_x = (1/12) * m * (d**2 + h**2) + 0.5**2 * m # 1/12*m*(h²+d²) + dx²*m
I_y = (1/12) * m * (d**2 + w**2) + 0.5**2 * m # 1/12*m*(w²+d²) + dx²*m
I_z = (1/12) * m * (w**2 + h**2) # 1/12*m*(w²+h²)

# fase 3.3
p.changeDynamics(barrierId, 0, localInertiaDiagonal=[I_x, I_y, I_z])

try:
    while True:
        pos, ori = p.getBasePositionAndOrientation(robotId)
        vel = p.getBaseVelocity(robotId)
        current_time = time.time() - start_time
        current_velocity = vel[0][1]

        roll, pitch, yaw = p.getEulerFromQuaternion(ori)

        # fase 4
        if pitch < -umbral_ori: # hacia arriba
            wheel_velocity = np.clip(wheel_velocity + umbral_vel, 0, max_wheel_velocity)

            for wheel_joint in wheels_joints:
                p.setJointMotorControl2(robotId, wheel_joint, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity, force=wheel_force)

        elif pitch > umbral_ori: # hacia abajo
            wheel_velocity = wheel_velocity * 0.6
            wheel_velocity = np.clip(wheel_velocity - umbral_vel, 0, max_wheel_velocity)

            for wheel_joint in wheels_joints_back: # joints 4 y 5
                p.setJointMotorControl2(robotId, wheel_joint, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity, force=wheel_force)

        else:
            for wheel_joint in wheels_joints:
                p.setJointMotorControl2(robotId, wheel_joint, p.VELOCITY_CONTROL, targetVelocity=11, force=25)

        if abs(pos[1] - last_y) >= dt:            
            data.append([current_time, pos[1], vel[0][1], wheel_velocity, wheel_force])
            last_y = pos[1]
        
        if pos[1] >= goal_pose_y:
            break

        time.sleep(1./240.)

    # --- Guardado de datos en CSV ---
    with open("Fase4.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Tiempo", "Posicion_Y", "Velocidad_Y", "Velocidad_Ruedas", "Fuerza_Ruedas"])
        writer.writerows(data)

except KeyboardInterrupt:
    pass

p.disconnect()
