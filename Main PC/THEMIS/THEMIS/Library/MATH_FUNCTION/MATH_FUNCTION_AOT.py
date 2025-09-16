#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2025 Westwood Robotics Corporation"
__date__      = "November 7, 2023"
__update__    = "August 25, 2025"
__project__   = "AOS"
__version__   = "0.1.0"
__status__    = "Production"

'''
Compile math function ahead of time (AOT) using Numba
'''

import numpy as np
from numba.pycc import CC


cc = CC('math_function')


@cc.export('norm', '(f8[:],)')
def norm(x):
    """
    compute the norm of a vector in R^3
    """
    return np.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])


@cc.export('sat', '(f8, f8, f8)')
def sat(x, x_min, x_max):
    """
    simple saturation function
    """
    if x > x_max:
        return x_max
    elif x < x_min:
        return x_min
    else:
        return x
    

@cc.export('hat', '(f8[:],)')
def hat(p):
    """
    convert R^3 to so(3)
    """
    return np.array([[    0, -p[2],  p[1]],
                     [ p[2],     0, -p[0]],
                     [-p[1],  p[0],     0]])


@cc.export('vee', '(f8[:,:],)')
def vee(S):
    """
    convert so(3) to R^3
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


@cc.export('hatexp', '(f8[:],)')
def hatexp(a):
    """
    convert rotation vector to rotation matrix
    """
    a = np.copy(a)
    ah = np.array([[    0, -a[2],  a[1]],
                   [ a[2],     0, -a[0]],
                   [-a[1],  a[0],     0]])
    ah2 = ah @ ah
    an2 = a @ a
    an  = np.sqrt(an2)
    if an < 1e-10:
        return np.eye(3)
    else:
        return np.eye(3) + np.sin(an) / an * ah + (1 - np.cos(an)) / an2 * ah2
    

@cc.export('logvee', '(f8[:,:],)')
def logvee(R):
    """
    convert rotation matrix to rotation vector
    """
    va = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.)
    if va > 1.:
        va = 1.
    elif va < -1:
        va = -1.
        
    theta = np.arccos(va)
    if np.abs(theta) < 1e-10:
        return np.zeros(3)
    else:
        logR = theta / 2. / np.sin(theta) * (R - R.T)
        return np.array([logR[2, 1], logR[0, 2], logR[1, 0]])


@cc.export('leftJac', '(f8[:],)')
def leftJac(a):
    """
    left Jacobian of SO(3)
    """
    a  = np.copy(a)
    ah = np.array([[    0, -a[2],  a[1]],
                   [ a[2],     0, -a[0]],
                   [-a[1],  a[0],     0]])
    ah2 = ah @ ah
    an2 = a @ a
    an  = np.sqrt(an2)

    if an < 1e-10:
        return np.eye(3)
    else:
        return np.eye(3) + (1 - np.cos(an)) / an2 * ah + (an - np.sin(an)) / an / an2 * ah2
    

@cc.export('rightJac', '(f8[:],)')
def rightJac(a):
    """
    right Jacobian of SO(3)
    """
    a  = np.copy(a)
    ah = np.array([[    0, -a[2],  a[1]],
                   [ a[2],     0, -a[0]],
                   [-a[1],  a[0],     0]])
    ah2 = ah @ ah
    an2 = a @ a
    an  = np.sqrt(an2)

    if an < 1e-10:
        return np.eye(3)
    else:
        return np.eye(3) - (1 - np.cos(an)) / an2 * ah + (an - np.sin(an)) / an / an2 * ah2
    

@cc.export('Hat', '(f8[:],)')
def Hat(v):
    Z = np.array([[    0, -v[2],  v[1],     0,     0,     0],
                  [ v[2],     0, -v[0],     0,     0,     0],
                  [-v[1],  v[0],     0,     0,     0,     0],
                  [    0, -v[5],  v[4],     0, -v[2],  v[1]],
                  [ v[5],     0, -v[3],  v[2],     0, -v[0]],
                  [-v[4],  v[3],     0, -v[1],  v[0],     0]])
    return Z


@cc.export('Hatstar', '(f8[:],)')
def Hatstar(v):
    Z = np.array([[    0, -v[2],  v[1],     0, -v[5],  v[4]],
                  [ v[2],     0, -v[0],  v[5],     0, -v[3]],
                  [-v[1],  v[0],     0, -v[4],  v[3],     0],
                  [    0,     0,     0,     0, -v[2],  v[1]],
                  [    0,     0,     0,  v[2],     0, -v[0]],
                  [    0,     0,     0, -v[1],  v[0],     0]])
    return Z


@cc.export('HatStar', '(f8[:],)')
def HatStar(v):
    Z1 = np.array([[    0, -v[2],  v[1],     0,     0,     0],
                   [ v[2],     0, -v[0],     0,     0,     0],
                   [-v[1],  v[0],     0,     0,     0,     0],
                   [    0, -v[5],  v[4],     0, -v[2],  v[1]],
                   [ v[5],     0, -v[3],  v[2],     0, -v[0]],
                   [-v[4],  v[3],     0, -v[1],  v[0],     0]])
    Z2 = np.array([[    0, -v[2],  v[1],     0, -v[5],  v[4]],
                   [ v[2],     0, -v[0],  v[5],     0, -v[3]],
                   [-v[1],  v[0],     0, -v[4],  v[3],     0],
                   [    0,     0,     0,     0, -v[2],  v[1]],
                   [    0,     0,     0,  v[2],     0, -v[0]],
                   [    0,     0,     0, -v[1],  v[0],     0]])
    return Z1, Z2


@cc.export('Rz', '(f8,)')
def Rz(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  -s, 0.],
                     [s,   c, 0.],
                     [0., 0., 1.]])


@cc.export('Ry', '(f8,)')
def Ry(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[ c,  0.,  s],
                     [ 0., 1., 0.],
                     [-s,  0.,  c]])


@cc.export('Rx', '(f8,)')
def Rx(t):
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1., 0., 0.],
                     [0.,  c, -s],
                     [0.,  s,  c]])


@cc.export('quat2rotm', '(f8[:],)')
def quat2rotm(Q):
    """
    convert quaternion to rotation matrix
    """
    Q  = Q / np.linalg.norm(Q)
    q  = Q[0:3]
    qh = np.array([[   0., -q[2],  q[1]],
                   [ q[2],    0., -q[0]],
                   [-q[1],  q[0],    0.]])
    q4 = Q[3]
    R  = (q4 * q4 - q @ q) * np.eye(3) + 2. * q4 * qh + 2. * np.outer(q, q)
    return R


@cc.export('exp_filter', '(f8, f8, f8)')
def exp_filter(history, present, weight):
    """
    exponential filter
    result = history * weight + present * (1. - weight)
    """
    return present + (history - present) * weight


@cc.export('linear_filter', '(f8, f8, f8)')
def linear_filter(history, present, increment):
    """
    linear filter
    """
    if history - present > increment:
        return history - increment
    elif present - history > increment:
        return history + increment
    else:
        return present


if __name__ == '__main__':
    cc.compile()