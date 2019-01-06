import pybullet as p
import time
import pybullet_data
import os
import numpy as np

from gym import spaces

Tsample_physics = 0.0001
UAV_DIR = os.path.join("Robots", "UAV")
uav_model = os.path.join(UAV_DIR, "quadrotor.urdf")
G = 9.81

# kp = np.matrix([[1, 0, 0, 0, 0, 0],
#                 [0, 1, 0, 0, 0, 0],
#                 [0, 0, 1, 0, 0, 0],
#                 [0, 0, 0, 1, 0, 0],
#                 [0, 0, 0, 0, 1, 0],
#                 [0, 0, 0, 0, 0, 1]])

# kd = np.matrix([[1, 0, 0, 0, 0, 0],
#                 [0, 1, 0, 0, 0, 0],
#                 [0, 0, 1, 0, 0, 0],
#                 [0, 0, 0, 1, 0, 0],
#                 [0, 0, 0, 0, 1, 0],
#                 [0, 0, 0, 0, 0, 1]])

kp = np.matrix([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 10, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])

kd = np.matrix([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])


def RotationMatrix(phi, theta, psi):
    """Returns the rotation matrix for three euler angles:
        phi - Roll
        theta - Pitch
        psi - Yaw
    """
    r1c1 = np.cos(psi) * np.cos(phi) - np.cos(theta) * np.sin(psi) * np.sin(phi)
    r1c2 = np.cos(psi) * np.sin(phi) + np.cos(theta) * np.sin(psi) * np.cos(phi)
    r1c3 = np.sin(psi) * np.sin(theta)

    r2c1 = -1 * np.sin(psi) * np.cos(phi) - np.cos(theta) * np.sin(phi) * np.cos(psi)
    r2c2 = -1 * np.sin(psi) * np.sin(phi) + np.cos(theta) * np.cos(phi) * np.cos(psi)
    r2c3 = np.cos(theta)

    r3c1 = np.sin(theta) * np.sin(phi)
    r3c2 = -1 * np.sin(theta) * np.cos(phi)
    r3c3 = np.cos(theta)

    r_matrix = np.matrix([[r1c1, r1c2, r1c3],
                          [r2c1, r2c2, r2c3],
                          [r3c1, r3c2, r3c3]])

    return r_matrix


def getFMatrices(state, mass, moment):
    Ix = moment[0]
    Iy = moment[1]
    Iz = moment[2]

    # F1 Matrix Definition
    f1 = np.matrix([[(np.sin(state[6]) * np.sin(state[8])) / mass, 0, 0, 0],
                    [-1 * (np.cos(state[6]) * np.sin(state[8])) / mass, 0, 0, 0],
                    [(np.cos(state[8])) / mass, 0, 0, 0],
                    [0, 1 / Ix, 0, 0],
                    [0, 0, 1 / Iy, 0],
                    [0, 0, 0, 1 / Iz]])

    # F2 matrix Definition
    f2 = np.matrix([[0],
                    [0],
                    [G],
                    [((state[9] * state[11] * (Iy - Iz)) / Ix)],
                    [-1 * ((state[7] * state[11] * (Iz - Ix)) / Iy)],
                    [(((Ix - Iy) * state[7] * state[9]) / Iz)]])

    return f1, f2


class UAVController:

    def __init__(self, start_pos, start_orient):
        self.m = 0.5
        self.last_error = None
        self.ModelFile = os.path.join(UAV_DIR, "quadrotor.urdf")
        self.Ix = 0.0023
        self.Iy = self.Ix
        self.Iz = 0.004
        self.G = -9.81
        self.initialState(start_pos, start_orient)

    def initialState(self, position, orientation):
        state = [0] * 12
        # Set state matrices
        state[0] = position[0]
        state[1] = 0
        state[2] = position[1]
        state[3] = 0
        state[4] = position[2]
        state[5] = 0
        state[6] = orientation[0]
        state[7] = 0
        state[8] = orientation[1]
        state[9] = 0
        state[10] = orientation[2]
        state[11] = 0
        self.state = state

    def GetActionFT(self, state_pos, state_orient, linV, angV):
        # Trajectory for hovering in the position (0, 0, 2)
        Xd = np.matrix([[0], [0], [2], [0], [0], [0]])
        dXd = np.matrix([[0], [0], [0], [0], [0], [0]])
        d2Xd = np.matrix([[0], [0], [0], [0], [0], [0]])

        rotation = RotationMatrix(state_orient[0], state_orient[1], state_orient[2])

        Wx_body, Wy_body, Wz_body = angV
        Wxyz_body = np.matrix([[Wx_body], [Wy_body], [Wz_body]])
        angV_world = np.matmul(rotation, Wxyz_body)

        Vx_body, Vy_body, Vz_body = linV
        Vxyz_body = np.matrix([[Vx_body], [Vy_body], [Vz_body]])
        linV_world = np.matmul(rotation, Vxyz_body)

        # Set state matrices
        self.state[0] = state_pos[0]
        self.state[1] = linV_world.item(0)
        self.state[2] = state_pos[1]
        self.state[3] = linV_world.item(1)
        self.state[4] = state_pos[2]
        self.state[5] = linV_world.item(2)
        self.state[6] = state_orient[0]
        self.state[7] = Wx_body
        self.state[8] = state_orient[1]
        self.state[9] = Wy_body
        self.state[10] = state_orient[2]
        self.state[11] = Wz_body

        x = np.matrix(
            [[self.state[0]], [self.state[2]], [self.state[4]], [self.state[6]], [self.state[8]], [self.state[10]]])
        x_dot = np.matrix(
            [[self.state[1]], [self.state[3]], [self.state[5]], [self.state[7]], [self.state[9]], [self.state[11]]])

        error = np.subtract(Xd, x)
        error_dot = np.subtract(dXd, x_dot)

        f1, f2 = getFMatrices(self.state, self.m, [self.Ix, self.Iy, self.Iz])

        Kpe = np.matmul(kp, error)
        Kde = np.matmul(kd, error_dot)
        v1 = np.add(d2Xd, Kpe)
        v2 = np.add(v1, Kde)
        v3 = np.add(v2, f2)
        inv = np.linalg.pinv(f1)

        print("Error")
        print(error)
        print("error_dot")
        print(error_dot)
        print("f2")
        print(f2)
        print("f1-inverse")
        print(inv)

        U = np.matmul(inv, v3)
        return U

    def DynamicModel(self, h, t, u):
        x = []
        xdot = []
        for i in range(12):
            x[i] = h[i]

        xdot[0] = x[1]  # Velocity in world-frame x-axis
        xdot[1] = (1 / self.m) * (np.sin(x[8]) * np.sin(x[6])) * u[0]  # Acceleration in world-frame x-axis
        xdot[2] = x[3]
        xdot[3] = (1 / self.m) * (-1 * np.sin(x[8]) * np.cos(x[6])) * u[0]
        xdot[4] = x[5]
        xdot[5] = self.G + (1 / self.m) * (np.cos(x[8])) * u[0]

        xdot[6] = (1 / np.sin(x[8])) * (np.sin(x[10]) * x[7] + np.cos(x[10]) * x[9])
        xdot[7] = ((self.Iy - self.Iz) / self.Ix) * x[9] * x[11] + u[1] / self.Ix

        xdot[8] = (1 / np.sin(x[8])) * (np.sin(x[8]) * np.cos(x[10]) * x[7] - np.sin(x[8]) * np.sin(x[10]) * x[9])
        xdot[9] = ((self.Iz - self.Ix) / self.Iy) * x[7] * x[11] + u[2] / self.Iy

        xdot[10] = (1 / np.sin(x[8])) * (
                    -1 * np.cos(x[8]) * np.sin(x[10]) * x[7] - np.cos(x[8]) * np.cos(x[10]) * x[9] + x[11])
        xdot[11] = ((self.Ix - self.Iy) / self.Iz) * x[7] * x[9] + u[3] / self.Iz

        return xdot


class TaskHover:

    def __init__(self):
        # Define action space
        max_force = 25
        max_torque = 25
        self.action_space = spaces.Box(
            np.array([-max_force, -max_force, -max_force, -max_torque, -max_torque, -max_torque]),
            np.array([max_force, max_force, max_force, max_torque, max_torque, max_torque])
        )

    def GenerateHover(self):
        X_d = np.matrix([[0],
                         [0],
                         [1.2],
                         [0]])

        dX_d = np.matrix([[0],
                          [0],
                          [0],
                          [0]])

        d2X_d = np.matrix([[0],
                           [0],
                           [0],
                           [0]])

        return X_d, dX_d, d2X_d


if __name__ == "__main__":
    SIM_COMPLETE = False

    physicsClient = p.connect(p.GUI)  # p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -1 * G)
    p.setTimeStep(Tsample_physics)
    p.setRealTimeSimulation(0)
    cubeStartPos = [1.0, 1.0, 1.0]
    cubeStartOrientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
    quadId = p.loadURDF(uav_model, cubeStartPos, cubeStartOrientation)
    planeId = p.loadURDF("plane.urdf")

    controller = UAVController(cubeStartPos, cubeStartOrientation)

    i = 0
    while i < 10:  # SIM_COMPLETE is not True:
        t = Tsample_physics * i
        xyz_pos, orient = p.getBasePositionAndOrientation(quadId)
        linearV, angularV = p.getBaseVelocity(quadId)

        action = controller.GetActionFT(xyz_pos, orient, linearV, angularV)
        # print(xyz_pos)
        print(action)
        if i % 100:
            print(xyz_pos)

        p.applyExternalForce(quadId, -1, [0, 0, action[0]], [action[1], action[2], action[3]], p.LINK_FRAME)
        p.stepSimulation()

        time.sleep(0.0001)
        i += 1

    cubePos, cubeOrn = p.getBasePositionAndOrientation(quadId)
    p.disconnect()