# ========================================================================
#   6 DOF simulator
#   Date: 8/09/2024
#   About:  This file contains the plotting functions for a given flight 
#           vector
# 
#   Author: Oliver Clements
#           Based upon Ben Dicksons Youtube series
#           Link: https://www.youtube.com/watch?v=hr_PqdkG6XY&list=PLcmbTy9X3gXs4JVXYucrMz5bJ4ZuaEGJ_&ab_channel=BenDickinson
# ========================================================================

# Libary imports
import matplotlib.pyplot as plt
import numpy as np
from typing import Union

# Module imports
from aircraft_models.models import VehicleData, Aircraft

def plot_aircraft_axis(t_s, x, vehicle_data: Union[VehicleData, Aircraft]):
    """ Plots the 6DoF data through out time"""
    fig, axes = plt.subplots(3, 3, figsize=(12,8))
    fig.suptitle(vehicle_data.name, fontsize=14, fontweight='bold') 

    # x axis velocity
    axes[0, 0].plot(t_s, x[0,:], color="blue")
    axes[0, 0].set_xlabel("Time [s]")
    axes[0, 0].set_ylabel("u (x) [m/s]")
    axes[0, 0].grid(True)

    # y axis velocity
    axes[0, 1].plot(t_s, x[1,:], color="green")
    axes[0, 1].set_xlabel("Time [s]")
    axes[0, 1].set_ylabel("v (y) [m/s]")
    axes[0, 1].grid(True)

    # z axis velocity
    axes[0, 2].plot(t_s, x[2,:], color="orange")
    axes[0, 2].set_xlabel("Time [s]")
    axes[0, 2].set_ylabel("w (z) [m/s]")
    axes[0, 2].grid(True)

    # Roll angle, phi
    axes[1, 0].plot(t_s, np.rad2deg(x[6,:]), color="blue")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("phi (Roll) [deg]")
    axes[1, 0].grid(True)

    # Pitch angle, theta
    axes[1, 1].plot(t_s, np.rad2deg(x[7,:]), color="green")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("theta (Pitch) [deg]")
    axes[1, 1].grid(True)

    # Yaw angle, psi
    axes[1, 2].plot(t_s, np.rad2deg(x[8,:]), color="orange")
    axes[1, 2].set_xlabel("Time [s]")
    axes[1, 2].set_ylabel("psi (Yaw) [deg]")
    axes[1, 2].grid(True)


    # Roll rate
    axes[2, 0].plot(t_s, np.rad2deg(x[3,:]), color="blue")
    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 0].set_ylabel("p (Roll) [deg/s]")
    axes[2, 0].grid(True)

    # Pitch rate
    axes[2, 1].plot(t_s, np.rad2deg(x[4,:]), color="green")
    axes[2, 1].set_xlabel("Time [s]")
    axes[2, 1].set_ylabel("q (Pitch) [deg/s]")
    axes[2, 1].grid(True)

    # Yaw rate
    axes[2, 2].plot(t_s, np.rad2deg(x[5,:]), color="orange")
    axes[2, 2].set_xlabel("Time [s]")
    axes[2, 2].set_ylabel("r (Yaw) [deg/s]")
    axes[2, 2].grid(True)

    

    plt.tight_layout()
    plt.show()


def plot_aircraft_motion(t_s, x, vehicle_data: Union[VehicleData, Aircraft]):
    """ Plots the aircraft motion such as altitude and north and east distances"""
    fig, axes = plt.subplots(2, 3, figsize=(10, 6))
    fig.suptitle(vehicle_data.name, fontsize=14, fontweight='bold', color='blue') 

    # North position p1^n_CM/T
    axes[0,0].plot(t_s, x[9,:], color='blue')
    axes[0,0].set_xlabel('Time [s]')
    axes[0,0].set_ylabel('North [m]')
    if np.linalg.norm(x[9,:]) < 1e-6:
        axes[0,0].set_ylim(-0.05,0.05)
    axes[0,0].grid(True)

    # East position p2^n_CM/T
    axes[0,1].plot(t_s, x[10,:], color='blue')
    axes[0,1].set_xlabel('Time [s]')
    axes[0,1].set_ylabel('East [m]')
    if np.linalg.norm(x[10,:]) < 1e-6:
        axes[0,1].set_ylim(-0.05,0.05)
    axes[0,1].grid(True)

    # Altitude
    axes[0,2].plot(t_s, -x[11,:], color='blue')
    axes[0,2].set_xlabel('Time [s]')
    axes[0,2].set_ylabel('Altitude [m]')
    if np.linalg.norm(x[11,:]) < 1e-6:
        axes[0,2].set_ylim(-0.05,0.05)
    axes[0,2].grid(True)

    # North vs East position p2^n_CM/T
    axes[1,0].plot(x[10,:], x[9,:], color='blue')
    axes[1,0].set_xlabel('East [s]')
    axes[1,0].set_ylabel('North [m]')
    if np.linalg.norm(x[9,:]) < 1e-6:
        axes[1,0].set_ylim(-0.05,0.05)
    if np.linalg.norm(x[10,:]) < 1e-6:
        axes[1,0].set_xlim(-0.05,0.05)
    axes[1,0].grid(True)

    # Altitude vs East position p2^n_CM/T
    axes[1,1].plot(x[10,:], -x[11,:], color='blue')
    axes[1,1].set_xlabel('East [s]')
    axes[1,1].set_ylabel('Altitude [m]')
    if np.linalg.norm(x[10,:]) < 1e-6:
        axes[1,1].set_xlim(-0.05,0.05)
    if np.linalg.norm(x[11,:]) < 1e-6:
        axes[1,1].set_ylim(-0.05,0.05)
    axes[1,1].grid(True)

    # Altitude vs North
    axes[1,2].plot(x[9,:], -x[11,:], color='blue')
    axes[1,2].set_xlabel('North [s]')
    axes[1,2].set_ylabel('Altitude [m]')
    if np.linalg.norm(x[9,:]) < 1e-6:
        axes[1,2].set_xlim(-0.05,0.05)
    if np.linalg.norm(x[11,:]) < 1e-6:
        axes[1,2].set_ylim(-0.05,0.05)
    axes[1,2].grid(True)

    plt.tight_layout()
    plt.show(block=False)

def plot_aerodynamic(t_s, x, vehicle_data: Union[VehicleData, Aircraft], alpha_rad, beta_rad, mach):
    """ Plots the aerodynamic data such as AoA and mach #"""
    fig, axes = plt.subplots(1, 3, figsize=(10, 6))
    fig.suptitle(vehicle_data.name, fontsize=14, fontweight='bold')

    # Angle of attack
    axes[0].plot(t_s, alpha_rad*180/3.14, color='magenta')
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Angle of Attack [deg]')
    axes[0].set_ylim(-90,90)
    axes[0].grid(True)


    # Angle of side slip
    axes[1].plot(t_s, beta_rad*180/3.14, color='magenta')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Angle of Side Slip [deg]')
    axes[1].set_ylim(-90,90)
    axes[1].grid(True)


    # Mach
    axes[2].plot(t_s, mach, color='magenta')
    axes[2].set_xlabel('Time [s]')
    axes[2].set_ylabel('Mach Number')
    axes[2].grid(True)

    plt.tight_layout()
    #plt.savefig('saved_figures/S1p4_Ex_5_Air_Data_Plot.png')
    plt.show()
