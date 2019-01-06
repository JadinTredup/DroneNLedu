import numpy as np

g = 9.81
m = 0.5
Ix = 0.0023
Iy = 0.0023
Iz = 0.004

kp = np.matrix([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

kd = np.matrix([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])


def UAVDynamicModel(h, t, u):
    x1  =   h[0]
    x2  =   h[1]
    x3  =   h[2]
    x4  =   h[3]
    x5  =   h[4]
    x6  =   h[5]
    x7  =   h[6]
    x8  =   h[7]
    x9  =   h[8]
    x10 =   h[9]
    x11 =   h[10]
    x12 =   h[11]

    dx1dt   =   x2
    dx2dt   =   (1/m)*(np.sin(x9)*np.sin(x7))*u[0]
    dx3dt   =   x4
    dx4dt   =   (1/m)*(-1*np.sin(x9)*np.cos(x7))*u[0]
    dx5dt   =   x6
    dx6dt   =   g+(1/m)*(np.cos(x9))*u[0]

    dx7dt   =   (1/np.sin(x9))*(np.sin(x11)*x8 + np.cos(x11)*x10)
    dx8dt   =   ((Iy-Iz)/Ix)*x10*x12 + u[1]/Ix

    dx9dt   =   (1/np.sin(x9))*(np.sin(x9)*np.cos(x11)*x8 - np.sin(x9)*np.sin(x11)*x10)
    dx10dt  =   ((Iz-Ix)/Iy)*x8*x12 + u[2]/Iy

    dx11dt  =   (1/np.sin(x9))*(-1*np.cos(x9)*np.sin(x11)*x8 - np.cos(x9)*np.cos(x11)*x10 + x12)
    dx12dt  =   ((Ix-Iy)/Iz)*x8*x10 + u[3]/Iz

    dydt = [dx1dt, dx2dt, dx3dt, dx4dt, dx5dt, dx6dt, dx7dt, dx8dt, dx9dt, dx10dt, dx11dt, dx12dt]

    return dydt