import pybullet as p
import time
import os
import numpy

class StandardUAV():

    def __init__(self):
        URDF_FN = os.path.join('SimulationSource', 'Robots', 'UAV', 'quadrotor.urdf')
        self.kp = np.matrix([1, 0,  0, 0, 0, 0],
                            [0, 1,  0, 0, 0, 0],
                            [0, 0, 10, 0, 0, 0],
                            [0, 0,  0, 1, 0, 0],
                            [0, 0,  0, 0, 1, 0],
                            [1, 0,  0, 0, 0, 1])
        self.kd = np.matrix([1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [1, 0, 0, 0, 0, 1])