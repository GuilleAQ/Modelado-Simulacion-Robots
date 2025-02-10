import pybullet as p
import pybullet_data
import time

SPHERE_RADIUS = 0.25

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 1]

robotId = p.loadURDF("urdf/ejercicio2.urdf", startPosition, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

# Definir condiciones iniciales
Zo = 3.0
v = 0.0
g = -9.8
dt = 1 / 240
t = 0.0

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))

for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
    t += dt

    # Calcular la velocidad usando la ecuación MRUA: v_f = v_0 + a t
    v = v + g * dt

    # Calcular la nueva posición usando: x = v_0 t + (1/2) a t^2
    z = Zo + (0 * t) + (0.5 * g * (t ** 2))
    Zo = z

    # Evitar que la esfera atraviese el suelo
    if z < SPHERE_RADIUS:
        z = SPHERE_RADIUS
        v = 0  # Detener el movimiento

    # Actualizar posición en PyBullet
    p.resetBasePositionAndOrientation(robotId, [0, 0, z], startOrientation)

p.disconnect()
