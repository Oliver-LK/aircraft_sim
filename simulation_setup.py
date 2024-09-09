# ========================================================================
#   6 DOF simulator
#   Date: 8/09/2024
#   About:  This file contains the simulation setup. Including atmosphere
#           data and initial conditions
# 
#   Author: Oliver Clements
#           Based upon Ben Dicksons Youtube series
#           Link: https://www.youtube.com/watch?v=hr_PqdkG6XY&list=PLcmbTy9X3gXs4JVXYucrMz5bJ4ZuaEGJ_&ab_channel=BenDickinson
# ========================================================================

# Library imports
import numpy as np
import math
import ussa1976

# Simulation time settings
TRIM_SIM_TIME           = 500 # (s)
PERTURBATION_SIM_TIME   = 500 # (s)

# =========================
#  Atmospheric data
# =========================
def init_atmos_data() -> dict:
    """ Grabs all the data from the 1976 standard atmosphere data"""
    atmos = ussa1976.compute()

    # Preload the atmospheric data (improves performance)
    alt_m       = atmos["z"].values
    rho_kgpm3   = atmos["rho"].values
    c_mps       = atmos["cs"].values
    g_mps2      = ussa1976.core.compute_gravity(alt_m)

    # Storing amphoteric data into single dictionary
    atmos_data = {  "alt_m"     : alt_m,\
                    "rho_kgpm3" : rho_kgpm3, \
                    "c_mps"     : c_mps, \
                    "g_mps2"    : g_mps2}
    
    return atmos_data

# =========================
# Initial conditions
# =========================
def init_x0_for_trim():
    """ Defines the initials conditions"""
    # Initial conditions
    # Change these as desired
    u0_bf_mps   =   200
    v0_bf_mps   =   -5
    w0_bf_mps   =   2
    p0_bf_rads  =   0 * math.pi/180
    q0_bf_rads  =   0 * math.pi/180
    r0_bf_rads  =   0 * math.pi/180
    phi0_rad    =   0 * math.pi/180
    theta0_rad  =   0 * math.pi/180
    psi0_rad    =   0 * math.pi/180
    p10_n_m     =   0
    p20_n_m     =   0
    p30_n_m     =   -1 * 8600      # Altitude (m)

    # Assign initial conditions into an array
    x0 = np.array([ u0_bf_mps,
                    v0_bf_mps,
                    w0_bf_mps,
                    p0_bf_rads,
                    q0_bf_rads,
                    r0_bf_rads,
                    phi0_rad,
                    theta0_rad,
                    psi0_rad,
                    p10_n_m,
                    p20_n_m,
                    p30_n_m,
                ])
    
    return x0


def init_x0_for_perturbation():
    """ Defines the initials conditions"""
    # Initial conditions
    # Change these as desired
    u0_bf_mps   =   209.46
    v0_bf_mps   =   0
    w0_bf_mps   =   -0.008
    p0_bf_rads  =   0 * math.pi/180
    q0_bf_rads  =   0 * math.pi/180
    r0_bf_rads  =   0 * math.pi/180
    phi0_rad    =   0 * math.pi/180
    theta0_rad  =   -0.002 * math.pi/180
    psi0_rad    =   0 * math.pi/180
    p10_n_m     =   0
    p20_n_m     =   0
    p30_n_m     =   -1 * 8670      # Altitude (m)

    # Assign initial conditions into an array
    x0 = np.array([ u0_bf_mps,
                    v0_bf_mps,
                    w0_bf_mps,
                    p0_bf_rads,
                    q0_bf_rads,
                    r0_bf_rads,
                    phi0_rad,
                    theta0_rad,
                    psi0_rad,
                    p10_n_m,
                    p20_n_m,
                    p30_n_m,
                ])
    
    print(x0)
    
    return x0


def give_trim_sim_runtime() -> float:
    """ Returns total sim runtime for the trim condition"""
    return TRIM_SIM_TIME


def give_perturbation_sim_time() -> float:
    """ Returns total sim runtime for the trim condition"""
    return PERTURBATION_SIM_TIME