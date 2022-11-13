#!/bin/env python3

import numpy as np
import matplotlib.pyplot as plt


x = [ -1, -0.5, 0, 0.5, 1 ]
y = [ -1, -0.5, 0, 0.5, 1 ]

u = [ 0, 0, 0, 0, 0 ]
v = [ 0, 0, 0, 0, 0 ]

y[0] *= 0.98
y[1] *= 1.10
y[3] *= 1.12
y[4] *= 0.9


for i in [1,2,3]:
    S = (x[i]-x[i-1]) / (x[i+1]-x[i-1])
    P = S * v[i-1] + 2.0
    v[i] = (S - 1.0) / P

    U = ((y[i+1] - y[i]) / (x[i+1] - x[i])) - ((y[i] - y[i-1]) / (x[i] - x[i-1]))
    u[i] = (6.0*U / (x[i+1] - x[i-1]) - S * u[i-1]) / P

    print(f'i={i}:: S={S} P={P} U={U} v[i]={v[i]} u[i]={u[i]}')

for k in [3,2,1,0]:
    v[k] = v[k] * v[k+1] + u[k]

for k in [0,1,2,3,4]:
    print(f'k={k}:: v[{k}] = {v[k]}')



def speval(X,j):
    i = j - 1
    h = x[j] - x[i]
    a = (x[j] - X) / h
    b = (X - x[i]) / h
    return a * y[i] + b * y[j] + ((a*a*a - a) * v[i] + (b*b*b - b) * v[j]) * (h*h) / 6.0


def spline(X):
    for i in [1,2,3]:
        if X < x[i]:
            return speval(X,i)
    return speval(X,4)


XX = np.linspace(-1.25, 1.25, 1000)
YY = np.zeros_like(XX)

for i in range(1000):
    YY[i] = spline(XX[i])

plt.plot(XX,YY)
plt.show()

