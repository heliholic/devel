#!/usr/bin/python3

import os
import sys
import csv
import math
import gzip

import numpy as np

from numpy import linalg as lin

from scipy import signal

import matplotlib.pyplot as plot

# Etocii curves
if True:
    Edr  = [  16, 16, 18, 21, 25, 31, 42, 62,125,250,250,250,250,250,250,250 ]
    Edl  = [  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 14, 15, 16 ]
    Odr  = [ 250,250,250,250, 25,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0 ]
    Odl  = [   5,  5,  4,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2 ]
    Obr  = [   0,  0,  0,  0,  0,  0,  2,  4, 30,250,250,250,250,250,250,250 ]
    Obl  = [   0,  0,  0,  0,  0,  0, 15, 40,100,150,200,250,250,250,250,250 ]
    Occ  = [   0,100,100,100, 90, 80, 68, 60, 54, 49, 45, 42, 39, 37, 35, 33 ]

# New curves
else:
    Edr  = [  16, 16, 18, 21, 25, 31, 42, 62,125,250,250,250,250,250,250,250 ]
    Edl  = [  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 14, 15, 16 ]
    Odr  = [ 250,250,250,250, 25,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5 ]
    Odl  = [   5,  5,  4,  3,  2,  2,  2,  2,  2,  1,  1,  1,  1,  1,  1,  1 ]
    Obr  = [   0,  0,  0,  0,  0,  0,  2,  4, 30,250,250,250,250,250,250,250 ]
    Obl  = [   0,  0,  0,  0,  0,  0, 15, 40,100,150,200,250,250,250,250,250 ]
    Occ  = [   0,100,100,100, 90, 80, 68, 60, 54, 49, 45, 42, 39, 37, 35, 33 ]



def getErrorDecayRate(coll):
    return 0

def getErrorDecayLimit(coll):
    return 0

def getOffsetDecayRate(coll):
    return 0

def getOffsetDecayLimit(coll):
    return 0

def getOffsetBleedRate(coll):
    return 0

def getOffsetBleedLimit(coll):
    return 0



period = 20
Fs = 1000

delta = 1 / Fs
count = period * Fs

Ki = 0.8
Ko = 0.4


COL = np.zeros(16)
Edt = np.zeros(16)
Odt = np.zeros(16)
Obt = np.zeros(16)

for n in range(0,16):
    COL[n] = n
    Edt[n] = 25 / (Edr[n] + 1)
    Odt[n] = 25 / (Odr[n] + 1)
    Obt[n] = 25 / (Obr[n] + 1)

if True:
    (fig,ax) = plot.subplots(1,1,figsize=(30,20),dpi=80)

    ax.plot(COL, Edt)
    #ax.plot(COL, Edl)
    #ax.plot(COL, Odt)
    #ax.plot(COL, Odl)
    #ax.plot(COL, Obt)
    #ax.plot(COL, Obl)

    plot.grid(which='both', axis='both')
    plot.show()



exit(0)


T = np.zeros(count)
I = np.zeros((count,16))
O = np.zeros((count,16))
S = np.zeros((count,16))


def limit(x, lim):
    if x > lim:
        return lim
    elif x < -lim:
        return -lim
    else:
        return x

for m in range(0,15):
    I[0][m] = 30
    O[0][m] = 30

for n in range(1,count):
    T[n] = n * delta
    for m in range(0,15):
        prev = I[n-1][m]
        rate = Edr[m] * 0.04
        maxr = Edl[m]
        decay = limit(prev * rate, maxr)
        I[n][m] = prev - decay * delta

        prev = O[n-1][m]
        rate = Odr[m] * 0.04
        maxr = Odl[m]
        decay = limit(prev * rate, maxr)
        O[n][m] = prev - decay * delta

        S[n][m] = I[n][m] * Ki + O[n][m] * Ko


if True:
    (fig,ax) = plot.subplots(2,2,figsize=(30,20),dpi=80)

    ax[0][0].plot(T, I)
    ax[0][1].plot(T, O)
    ax[1][0].plot(T, S)

    plot.grid(which='both', axis='both')
    plot.show()

