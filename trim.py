# ========================================================================
#   6 DOF simulator
#   Date: 8/09/2024
#   About:  This file contains the calls for finding the trim data for the 
#           the flight. This has been set for the mustang
# 
#   Author: Oliver Clements
#           Based upon Ben Dicksons Youtube series
#           Link: https://www.youtube.com/watch?v=hr_PqdkG6XY&list=PLcmbTy9X3gXs4JVXYucrMz5bJ4ZuaEGJ_&ab_channel=BenDickinson
# ========================================================================

# Libary imports
import numpy as np
import math
import time

# Module imports
from intergration import foward_euler
from simulation_setup import init_atmos_data, init_x0_for_trim, give_trim_sim_runtime
from aircraft_simulator import flat_earth_eom
from aircraft_models.models import *
from plots import plot_aerodynamic, plot_aircraft_axis, plot_aircraft_motion

# Record the start time
start_time = time.time()

# Settings
DO_PLOT = True

# ============================================
# Init of simulation
# ============================================

# Simulation times
t0_s = 0        # Start time
tf_s =  give_trim_sim_runtime()
h_s = 0.1       # Time step

# Get the atmospheric data
atmos_data =  init_atmos_data()

#  Defining the vehicle
vehicle_data = get_Mustang()

if type(vehicle_data) == Aircraft:
    print("Using Aircraft functions")

elif type(vehicle_data) == VehicleData:
    print("Using basic vehicle data functions")

# Initial conditions vector
x0 = init_x0_for_trim()

# Make it a colum vector
nx0 = x0.size


# ============================================
# Run the simulation
# ============================================

# Allocation for solution
t_s = np.arange(t0_s, tf_s + h_s, h_s); nt_s = t_s.size
x = np.zeros((nx0, nt_s), dtype=float)

# Assign the initial conditions
x[:,0] = x0

# Perform the numerical integration
t_s, x = foward_euler(flat_earth_eom, t_s, x, h_s, vehicle_data, atmos_data)

# Defining other data variables
true_air_speed_mps = np.zeros((nt_s, 1))
altitude_m  = np.zeros((nt_s, 1))
cs_mps      = np.zeros((nt_s, 1))
rho_kgpm3_c = np.zeros((nt_s, 1))
alpha_rad   = np.zeros((nt_s, 1))
beta_rad    = np.zeros((nt_s, 1))
mach        = np.zeros((nt_s, 1))

for i, element in enumerate(t_s):
    true_air_speed_mps[i, 0] = math.sqrt(x[0,i] ** 2 + x[1, i] ** 2 + x[2, i] ** 2)
    altitude_m[i, 0] = -x[11, i]
    cs_mps[i, 0] = np.interp(altitude_m[i, 0], atmos_data["alt_m"], atmos_data["c_mps"])
    rho_kgpm3_c[i, 0] = np.interp(altitude_m[i, 0], atmos_data["alt_m"], atmos_data["rho_kgpm3"])
    mach[i, 0] = true_air_speed_mps[i, 0] / cs_mps[i, 0]


    # angle of attack
    if x[0, i] == 0 and x[2, i] == 0:
        w_over_v = 0

    else:
        w_over_v = x[2,i] / x[0,i]

    alpha_rad[i, 0] = math.atan(w_over_v)

    # Angle of sideslip
    if x[1, i] == 0 and true_air_speed_mps[i, 0] == 0:
        v_over_VT = 0

    else:
        v_over_VT = x[1,i] / true_air_speed_mps[i, 0]

    beta_rad[i, 0] = math.asin(v_over_VT)


# Record the end time
end_time = time.time()
execution_time = end_time - start_time

print(f"Execution time: {execution_time:.5f} seconds")

if DO_PLOT == True:
    plot_aircraft_axis(t_s, x, vehicle_data)
    plot_aircraft_motion(t_s, x, vehicle_data)
    plot_aerodynamic(t_s, x, vehicle_data, alpha_rad, beta_rad, mach)
