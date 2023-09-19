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


## Lotto numbers

Edr  = [  16, 16, 18, 21, 25, 31, 42, 62,125,250,250,250,250,250,250,250 ]
Edl  = [  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 14, 15, 16 ]
Odr  = [ 250,250,250,250, 25,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0 ]
Odl  = [   5,  5,  4,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2 ]
Obr  = [   0,  0,  0,  0,  0,  0,  2,  4, 30,250,250,250,250,250,250,250 ]
Obl  = [   0,  0,  0,  0,  0,  0, 15, 40,100,150,200,250,250,250,250,250 ]
Occ  = [   0,100,100,100, 90, 80, 68, 60, 54, 49, 45, 42, 39, 37, 35, 33 ]


## Functions

def minmax(x, low, high):
    if x > high:
        return high
    elif x < low:
        return low
    else:
        return x

def limit(x, lim):
    if x > lim:
        return lim
    elif x < -lim:
        return -lim
    else:
        return x

def lookup(table, x):
    bins = len(table) - 1
    index = minmax(math.floor(x), 0, bins-1)
    a = table[index + 0]
    b = table[index + 1]
    dy = b - a
    dx = x - index
    y = a + dx * dy
    return y

def transition(x,lx,hx,ly,hy):
    if x < lx:
        return ly
    elif x > hx:
        return hy
    else:
        dx = hx - lx
        dy = hy - ly
        y = ly + dy * ((x - lx) / dx)
        return y


def getErrorDecayRate(value, lotto=False):
    if lotto:
        y = lookup(Edr, value)
    else:
        y = 25 / transition(value, 1, 9, 1.6, 0.1)
    return y

def getErrorDecayTime(value, lotto=False):
    if lotto:
        y = 25 / minmax(lookup(Edr, value), 0.1, 250)
    else:
        y = transition(value, 1, 9, 1.6, 0.1)
    return y

def getErrorDecayLimit(value, lotto=False):
    if lotto:
        y = lookup(Edl, value)
    else:
        y = transition(value, 0, 1, 0, 1)
    return y

def getOffsetDecayRate(value, lotto=False):
    if lotto:
        y = lookup(Odr, value)
    else:
        y = transition(value, 3, 6, 250, 0.1)
    return y

def getOffsetDecayTime(value, lotto=False):
    if lotto:
        y = 25 / minmax(lookup(Odr, value), 0.1, 250)
    else:
        y = 25 / transition(value, 3, 6, 250, 0.1)
    return y

def getOffsetDecayLimit(value, lotto=False):
    if lotto:
        y = lookup(Odl, value)
    else:
        y = transition(value, 0, 1, 0, 1)
    return y

def getOffsetBleedRate(value, lotto=False):
    if lotto:
        y = lookup(Obr, value)
    else:
        y = 25 / transition(value, 1, 2, 1, 2)
    return y

def getOffsetBleedTime(value, lotto=False):
    if lotto:
        y = 25 / minmax(lookup(Obr, value), 0.1, 250)
    else:
        y = transition(value, 0, 1, 0, 1)
    return y

def getOffsetBleedLimit(value, lotto=False):
    if lotto:
        y = lookup(Obl, value)
    else:
        y = transition(value, 0, 1, 0, 1)
    return y



def plotEtociiCurves():

    N = 1000

    COL = np.zeros(N)

    EDT1 = np.zeros(N)
    EDT2 = np.zeros(N)
    EDR1 = np.zeros(N)
    EDR2 = np.zeros(N)
    EDL1 = np.zeros(N)
    EDL2 = np.zeros(N)

    ODT1 = np.zeros(N)
    ODT2 = np.zeros(N)
    ODR1 = np.zeros(N)
    ODR2 = np.zeros(N)
    ODL1 = np.zeros(N)
    ODL2 = np.zeros(N)

    OBT1 = np.zeros(N)
    OBT2 = np.zeros(N)
    OBR1 = np.zeros(N)
    OBR2 = np.zeros(N)
    OBL1 = np.zeros(N)
    OBL2 = np.zeros(N)

    for n in range(0,N):
        C = 16 * n / N

        COL[n] = C

        EDT1[n] = getErrorDecayTime(C, False)
        EDR1[n] = getErrorDecayRate(C, False)
        ODT1[n] = getOffsetDecayTime(C, False)
        ODR1[n] = getOffsetDecayRate(C, False)
        OBT1[n] = getOffsetBleedTime(C, False)
        OBR1[n] = getOffsetBleedRate(C, False)

        EDL1[n] = getErrorDecayLimit(C, False)
        ODL1[n] = getOffsetDecayLimit(C, False)
        OBL1[n] = getOffsetBleedLimit(C, False)

        EDT2[n] = getErrorDecayTime(C, True)
        EDR2[n] = getErrorDecayRate(C, True)
        ODT2[n] = getOffsetDecayTime(C, True)
        ODR2[n] = getOffsetDecayRate(C, True)
        OBT2[n] = getOffsetBleedTime(C, True)
        OBR2[n] = getOffsetBleedRate(C, True)

        EDL2[n] = getErrorDecayLimit(C, True)
        ODL2[n] = getOffsetDecayLimit(C, True)
        OBL2[n] = getOffsetBleedLimit(C, True)

    (fig,ax) = plot.subplots(1,1,figsize=(30,20),dpi=80)

    ax.plot(COL, ODR2)

    plot.grid(which='both', axis='both')
    plot.show()


def plotDecay():

    period = 20
    Fs = 1000

    delta = 1 / Fs
    count = period * Fs

    Ki = 0.8
    Ko = 0.4

    T = np.zeros(count)
    I = np.zeros((count,16))
    O = np.zeros((count,16))
    S = np.zeros((count,16))


    for m in range(0,15):
        I[0][m] = 30
        O[0][m] = 30

    for n in range(1,count):
        T[n] = n * delta
        for m in range(0,15):
            C = m
            prev = I[n-1][m]
            rate = getErrorDecayRate(C) * 0.04
            maxr = getErrorDecayLimit(C)
            decay = limit(prev * rate, maxr)
            I[n][m] = prev - decay * delta

            prev = O[n-1][m]
            rate = getOffsetDecayRate(C) * 0.04
            maxr = getOffsetDecayLimit(C)
            decay = limit(prev * rate, maxr)
            O[n][m] = prev - decay * delta

            S[n][m] = I[n][m] * Ki + O[n][m] * Ko

    (fig,ax) = plot.subplots(2,2,figsize=(30,20),dpi=80)

    ax[0][0].plot(T, I)
    ax[0][1].plot(T, O)
    ax[1][0].plot(T, S)

    plot.grid(which='both', axis='both')
    plot.show()


plotEtociiCurves()

