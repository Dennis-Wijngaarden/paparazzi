#!/usr/bin/env python3

import pandas as pd
import scipy as sp
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

# Imports for opening a file from a file dialog
import tkinter as tk
from tkinter import filedialog

# Do a linear least squares fit of the data
def fit_axis(x,y, axis):
    c = np.linalg.lstsq(x, y)[0]
    print('Fit of axis ' + axis + ': ' + str(c))
    plt.figure()
    plt.plot(t,y)
    plt.plot(t,np.dot(x,c))
    plt.ylabel(axis + ' dotdot[rad/s^3]')
    plt.xlabel('t [s]')
    return c

# Sample frequency
sf = 500
#First order actuator dynamics constant (discrete, depending on sf)
fo_c = 0.04

root = tk.Tk()
root.withdraw()

# First load csv log
#log_path = filedialog.askopenfilename(filetypes=[("csv log files", ".csv")])
log_path = "/home/dennis/Documents/Altura/altura_logs/21_01_15_cyberzoo_pid/21_01_15__15_01_53_SD_no_GPS.csv"

# Open csv as pandas dataframe
df = pd.read_csv(log_path)

# Length of data dict
N = len(df["p"].tolist())

# create data lists
t = np.arange(N)/sf
gyro = np.array([df["p"].to_numpy(), df["q"].to_numpy(), df["r"].to_numpy()]).T
accel = np.array([df["accelz"].to_numpy()]).T
cmd = np.array([df["cmd_thrust"].to_numpy(), df["cmd_roll"].to_numpy(), df["cmd_pitch"].to_numpy(), df["cmd_yaw"].to_numpy()]).T

# Actuator dynamics
cmd_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], cmd, axis=0)

# Define Butterworth noise filter
b, a = sp.signal.butter(2, 2.5/(sf/2), 'low', analog=False)

# Filter stabilization command with actauator dynamics to get an estimate of the actuator state
gyro_f = sp.signal.lfilter(b, a, gyro, axis=0)
cmd_af = sp.signal.lfilter(b, a, cmd_a, axis=0)
accel_f = sp.signal.lfilter(b, a, accel, axis=0)

# derivatives
dgyro_f = np.vstack((np.zeros((1,3)), np.diff(gyro_f,1,axis=0)))*sf
ddgyro_f = np.vstack((np.zeros((1,3)), np.diff(dgyro_f,1,axis=0)))*sf
daccel_f = np.vstack((np.zeros((1,1)), np.diff(accel_f,1,axis=0)))*sf
dcmd_af = np.vstack((np.zeros((1,4)), np.diff(cmd_af,1,axis=0)))*sf
ddcmd_af = np.vstack((np.zeros((1,4)), np.diff(dcmd_af,1,axis=0)))*sf

# Estimation of the control effectiveness
c = fit_axis(dcmd_af[:,[1]], ddgyro_f[:,[0]], 'p')
c = fit_axis(dcmd_af[:,[2]], ddgyro_f[:,[1]], 'q')
g1_r = fit_axis(dcmd_af[:,[3]], ddgyro_f[:,[2]], 'r')
g2_r = fit_axis(ddcmd_af[:,[3]], ddgyro_f[:,[2]], 'r2')
c = fit_axis(dcmd_af[:,[0]], daccel_f[:,[0]], 'accelz')

plt.figure()
plt.plot(t,ddgyro_f[:,[2]])
plt.plot(t,dcmd_af[:,[3]]*g1_r + ddcmd_af[:,[3]]*g2_r)
plt.ylabel('r' + ' dotdot[rad/s^3]')
plt.xlabel('t [s]')

plt.figure()
plt.plot(t,dgyro_f[:,[2]])
plt.ylabel('r' + ' dot[rad/s^2]')
plt.xlabel('t [s]')

plt.show()