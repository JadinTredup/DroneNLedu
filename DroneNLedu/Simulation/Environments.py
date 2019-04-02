import pybullet as p
import time
import os
import numpy

class StandardEnvironment():
    """
    This class represents a basic environment in the Pybullet simulation with standard gravity
    and no pre-loaded robot models or environmental interferences.
    """

    def __init__(self):
        self.G = 9.81
        self.physics_sample_time = 0.0001
        self.RUNNING = False
        self.agents = []


    def insert_agent(self, agent=None):


    def start_client(self):
        self.client = p.connect(p.GUI) if (self._gui is True) else p.connect(p.DIRECT)
        p.setGravity(0, 0, -1 * self.G)
        p.setTimeStep(self.physics_sample_time)
        p.setRealTimeSimulation(self._real_time)


    def run(self, graphical=True, real_time=0):
        """

        :param graphical:
        :param real_time:
        :return:
        """
        self._gui = graphical
        self._real_time = real_time
        self.start_client()
