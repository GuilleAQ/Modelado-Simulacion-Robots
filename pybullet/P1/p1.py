import pybullet as p
import pybullet_data
import time
import numpy as np
import csv

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Establecer la posición y orientación de la cámara
camera_target_position = [3, 9.5, 4]  # Punto al que mira la cámara
camera_distance = 12  # Distancia de la cámara desde el objetivo
camera_yaw = 90  # Rotación horizontal
camera_pitch = -35  # Ángulo vertical
camera_roll = 0  # Inclinación de la cámara

p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                             cameraYaw=camera_yaw,
                             cameraPitch=camera_pitch,
                             cameraTargetPosition=camera_target_position)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 3.1415/2]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPositionRobot = [0, 0, 1]
startPositionRamp = [0, 10, 0.75]
startPositionBarrier = [-1.25, 17, 0.5]
startPositionGoal = [0, 20, 0.1]

robotId = p.loadURDF("husky/husky.urdf", startPositionRobot, startOrientation)
rampId = p.loadURDF("urdf/ramp.urdf", startPositionRamp, startOrientation)
barrierId = p.loadURDF("urdf/barrier.urdf", startPositionBarrier, startOrientation)
goalId = p.loadURDF("urdf/goal.urdf", startPositionGoal, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))


wheels_joints = [2 , 3, 4, 5]
target_velocity = 10  # Velocidad inicial de las ruedas
# p.setJointMotorControlArray(robotId, wheels_joints, p.VELOCITY_CONTROL, targetVelocities=[target_velocity] * 4)
# # body_link_vel = p.addUserDebugParameter("body_link_vel", -0.5, 0.5, 0)
# body_link_pos = p.addUserDebugParameter(" body_link_pos", -1, 1, 0)

# top_link_vel = p.addUserDebugParameter(" top_link_vel", -1, 1, 0)

# Inicializar almacenamiento de datos
data = []
last_y = 0  # Para verificar avance de 0.01 metros
start_time = time.time()

p.setRealTimeSimulation(1)

# pos, orn = p.getBasePositionAndOrientation(planeId)
pos, orn = p.getBasePositionAndOrientation(goalId)
euler = p.getEulerFromQuaternion(orn)
#print(f"Orientación en Euler: {euler}")
# print(f"Posición en Euler: {pos}")
GOAL_POSE_Y = pos[1]
# print(f"Posición en Euler: {GOAL_POSE_Y}")


wheel_velocity = 11
wheel_force = 25

lateralFriction=0.93
spinningFriction=0.005
rollingFriction=0.003

friction_coefs = [lateralFriction, spinningFriction, rollingFriction]

# Aplicar velocidad y fuerza a las ruedas
# p.setJointMotorControlArray(robotId, wheels_joints, p.VELOCITY_CONTROL, 
#                             targetVelocities=[wheel_velocity] * 4, 
#                             forces=[wheel_force] * 4)


# p.setJointMotorControl2(robotId, 2, p.TORQUE_CONTROL, force=wheel_force)
# p.setJointMotorControl2(robotId, 3, p.TORQUE_CONTROL, force=wheel_force)
# p.setJointMotorControl2(robotId, 4, p.TORQUE_CONTROL, force=wheel_force)
# p.setJointMotorControl2(robotId, 5, p.TORQUE_CONTROL, force=wheel_force)
for wheel_joint in wheels_joints:
    print(f"Joint número: {wheel_joint}")
    # fase 3.1
    p.setJointMotorControl2(robotId, wheel_joint, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity, force=wheel_force)

    # fase 3.2
    p.changeDynamics(robotId, wheel_joint, lateralFriction=lateralFriction, 
                     spinningFriction=spinningFriction, rollingFriction=rollingFriction)



# parámetros de la barrera
m = 5.0
d = 0.2
h = 0.2
w = 2.0

# Calcular momentos de inercia
I_x = (1/12) * m * (d**2 + h**2) + 0.5**2 * m # 1/12*m*(h²+d²) + dx²*m
I_y = (1/12) * m * (d**2 + w**2) + 0.5**2 * m # 1/12*m*(w²+d²) + dx²*m
I_z = (1/12) * m * (w**2 + h**2) # 1/12*m*(w²+h²)

print(f"x: {I_x}, y: {I_y}, z: {I_z}")
p.changeDynamics(barrierId, 0, localInertiaDiagonal=[I_x, I_y, I_z])


# Parámetros del controlador PD
Kp = 0.75  # Ganancia proporcional
Kd = 0.25  # Ganancia derivativa
previous_error = 0
previous_time = time.time()

# Inicializar almacenamiento de datos
data = []
last_y = 0

v_desired = 2.0  # m/s
r_husky = 0.165  # m

# w = v/r [rad/s]
omega_desired = v_desired / r_husky # rad/s

while True:
    # # bl_vel = p.readUserDebugParameter(body_link_vel)
    # bl_pos = p.readUserDebugParameter(body_link_pos)
    # # p.setJointMotorControl2(robotId, body_joint, p.VELOCITY_CONTROL, targetVelocity=bl_vel)
    # p.setJointMotorControl2(robotId, body_joint, p.POSITION_CONTROL, targetPosition=bl_pos)
    # p.setJointMotorControlArray(robotId, wheels_joints, p.VELOCITY_CONTROL, targetVelocities=[10,10,10,10])
    

    # tl_vel = p.readUserDebugParameter(top_link_vel)
    # p.setJointMotorControl2(robotId, top_joint, p.VELOCITY_CONTROL, targetVelocity=tl_vel)


    # # Obtener la posición y orientación del coche
    # pos, orn = p.getBasePositionAndOrientation(robotId)
    
    # # Convertir la orientación (cuaternión) a ángulos de Euler
    # euler = p.getEulerFromQuaternion(orn)
    
    # # Calcular la posición de la cámara (justo en la cabina del coche)
    # camera_offset = [0.2, 0, 0.5]  # Ajuste para la vista en primera persona
    # rotation_matrix = np.array([
    #     [np.cos(euler[2]), -np.sin(euler[2]), 0],
    #     [np.sin(euler[2]), np.cos(euler[2]), 0],
    #     [0, 0, 1]
    # ])
    
    # camera_position = np.array(pos) + rotation_matrix @ np.array(camera_offset)
    
    # # Girar la cámara 90° sobre el eje Z
    # cameraYaw = np.degrees(euler[2]) - 90  # Ajustar para girar la vista

    # # Configurar la cámara en primera persona con la rotación aplicada
    # p.resetDebugVisualizerCamera(
    #     cameraDistance=0.1,  # Cámara pegada al coche
    #     cameraYaw=cameraYaw,  # Girada 90° a la derecha
    #     cameraPitch=0,  # Vista horizontal
    #     cameraTargetPosition=camera_position
    # )

    # pos, _ = p.getBasePositionAndOrientation(robotId)
    # vel = p.getBaseVelocity(robotId)
    # current_time = time.time() - start_time
    
    # if abs(pos[1] - last_y) >= 0.01:  # Registro cada 0.01 metros avanzados en Y
    #     wheel_velocities = [target_velocity] * 4  # Todas las ruedas tienen la misma velocidad
    #     wheel_force = [0] * 4  # PyBullet no tiene una API directa para obtener la fuerza aplicada
        
    #     data.append([current_time, pos[1], vel[0][1], wheel_velocities[0], wheel_force[0]])
    #     last_y = pos[1]


    # Obtener la velocidad lineal actual en el eje Y
    vel = p.getBaseVelocity(robotId)[0][1]
    
    # Calcular error y derivada del error
    # error = abs(v_desired - vel)
    error = v_desired - vel
    current_time = time.time()
    dt = np.clip(current_time - previous_time, 0.1, None)
    
    derivative = (error - previous_error) / dt
    
    # Control PD
    control_signal = Kp * error + Kd * derivative

    # Convertir a velocidad angular

    omega = np.clip(vel / r_husky, 12, 2*omega_desired)
    print(f"vel_lineal: {control_signal}, vel_angular: {omega}, real vel: {vel}\n\n")
    
    # Aplicar control a las ruedas
    for wheel in wheels_joints:
        p.setJointMotorControl2(robotId, wheel, p.VELOCITY_CONTROL, targetVelocity=omega, force=wheel_force)

    # Actualizar valores previos
    previous_error = error
    previous_time = current_time

    pos = p.getBasePositionAndOrientation(robotId)[0]

    if abs(pos[1] - last_y) >= 0.01:  # Registro cada 0.01 metros avanzados en Y        
        # Guardar datos
        data.append([current_time, pos[1], vel, control_signal, wheel_force])
        last_y = pos[1]
    
    if pos[1] >= GOAL_POSE_Y:  # Terminar la simulación al llegar al final del recorrido
        break

    # p.setRealTimeSimulation(1)
    time.sleep(1./240.)

# p.disconnect()

# Guardar datos en CSV
with open("metricas_husky_f4.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Tiempo", "Posicion_Y", "Velocidad_Y", "Velocidad_Ruedas", "Fuerza_Ruedas"])
    writer.writerows(data)

p.disconnect()
print("Datos guardados en metricas_husky.csv")
