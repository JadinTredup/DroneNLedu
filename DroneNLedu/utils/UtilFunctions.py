import socket


def checksum(data):
    return_data = (data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5]) & 0xFF
    return return_data


def sendMessageHS(socket, message, recv_size):
    socket.send(message)
    for i in range(0, recv_size):
        data = socket.recv(1)
