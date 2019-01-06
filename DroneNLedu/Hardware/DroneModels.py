import socket
import logging
import threading
import time
import numpy as np
import multiprocessing as mp

import DroneNLedu.DroneConfig
from DroneNLedu.utils.UtilFunctions import checksum, sendMessageHS


class CX10WD(object):

    def __init__(self):
        self._ip = '172.16.10.1'
        self._tcp_port = 8888
        self._udp_port = 8895
        self.is_connected = False

    #########################################
    #                                       #
    #   Functions for Socket connections    #
    #                                       #
    #########################################

    def connect(self):
        self.connectTCP()
        self.connectUDP()
        self.is_connected = True

    def connectTCP(self):
        print("Starting Handshake...")
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self._ip, self._tcp_port))
        sendMessageHS(self.tcp_socket, DroneConfig.MESSAGE_1, 106)
        print('Message #1 Confirmed')

        sendMessageHS(self.tcp_socket, DroneConfig.MESSAGE_2, 106)
        print("Message #2 Confirmed")

        sendMessageHS(self.tcp_socket, DroneConfig.MESSAGE_3, 170)
        print("Message #3 Confirmed")

        sendMessageHS(self.tcp_socket, DroneConfig.MESSAGE_4, 106)
        print("Message #4 Confirmed")

        sendMessageHS(self.tcp_socket, DroneConfig.MESSAGE_5, 106)
        print("Message #5 Confirmed")

        print("Handshake done!")

    def connectUDP(self):
        print("Starting Drone...")
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.connect((self._ip, self._udp_port))
        self.udp_socket.send(DroneConfig.INIT_COMMAND)
        print("Drone Started!")

    def disconnect(self):
        print("Disconnecting...")
        self.udp_socket.close()
        self.tcp_socket.close()
        print("Disconnected!")

    def testCommandProcess(self):
        droneCmd = DroneConfig.INIT_COMMAND
        droneCmd[1] = 127
        droneCmd[2] = 125
        droneCmd[3] = 15
        droneCmd[4] = 127
        droneCmd[5] = 0x01
        droneCmd[6] = checksum(droneCmd)
        self.udp_socket.send(droneCmd)

        droneCmd[5] = 0x00
        droneCmd[3] = 50

        for i in range(0, 50):
            droneCmd = DroneConfig.INIT_COMMAND
            droneCmd[1] = 127
            droneCmd[2] = 127
            droneCmd[3] = 50
            droneCmd[4] = 127
            droneCmd[5] = 0x00
            droneCmd[6] = checksum(droneCmd)
            self.udp_socket.send(droneCmd)

            if i == 45:
                droneCmd = DroneConfig.INIT_COMMAND
                droneCmd[1] = 127
                droneCmd[2] = 127
                droneCmd[3] = 0
                droneCmd[4] = 127
                droneCmd[5] = 0x02
                droneCmd[6] = checksum(droneCmd)
                self.udp_socket.send(droneCmd)

            print(droneCmd)

    def PerformAction(self, control):
        command_list = []
        droneCmd = DroneConfig.INIT_COMMAND
        self.udp_socket.send(droneCmd)

        while True:
            if not control.empty():
                command = control.get()
                print(command)
                # droneCmd[1] = bytes([int(command[0])])
                # droneCmd[2] = bytes([int(command[1])])
                # droneCmd[3] = bytes([int(command[2])])
                # droneCmd[4] = bytes([int(command[3])])
                # droneCmd[5] = bytes([int(command[4])])
                # droneCmd[6] = checksum(droneCmd)
                droneCmd[1] = int(command[0])
                droneCmd[2] = int(command[1])
                droneCmd[3] = int(command[2])
                droneCmd[4] = int(command[3])
                droneCmd[5] = int(command[4])
                droneCmd[6] = checksum(droneCmd)

            # print(droneCmd)
            self.udp_socket.send(droneCmd)
            droneCmd[5] = 0x00
            command_list.append(droneCmd)

            if droneCmd[5] == 2:
                break

            time.sleep(0.001)

    def HeartBeat(self):
        while True:
            print('Heartbeat Sent...')
            # Send heartbeat code goes here
            print("The Drone is alive!")
            time.sleep(5)
