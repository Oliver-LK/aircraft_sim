# ========================================================================
#   6 DOF simulator
#   Date: 8/09/2024
#   About:  This file contains all the calculations required for the 6Dof
#           aircraft simulator. This has not been optimized
# 
#   Author: Oliver Clements
#           Based upon Ben Dicksons Youtube series
#           Link: https://www.youtube.com/watch?v=hr_PqdkG6XY&list=PLcmbTy9X3gXs4JVXYucrMz5bJ4ZuaEGJ_&ab_channel=BenDickinson
# ========================================================================


#  -= Assumptions =-
# - Earth has infinite radius (therefore flat)
# - Gravity is constant and is tangential to the earths surface
# - Aircraft mass is constant
# - Aircraft is a rigid body


# -= Variable naming
# {variable}_{reference frame}_{units}
# Example: v_b_mps
#       This is the velocity in the x direction
#       in the aircraft body reference frame
#       with units of meters per second


# Library imports
import numpy as np
import math
from typing import Union
from aircraft_models.models import VehicleData, Aircraft

# Settings
PRINT_READ_OUTS = False
DO_PERTURBATION = True

PERTURBATION_TIME = 100


G = 9.81    # If constant gravity is required


def flat_earth_eom(t: float, x, vehicle_data: Union[VehicleData, Aircraft] , atmos_data: dict) -> np.ndarray:
    """
    Six degree of freedom (6DoF) equations of motion for an aircraft.
    Designed to be iterated upon by numerical integration methods.
    
    Arguments
        t: time (s)
        x: state vector
        vehicle_data: aircraft model data
        atmos_data: atmospheric data from the 1976 data base

    returns -> dx: time derivative of the state vector
    """

    # Time derivative of state vector
    dx = np.empty((12,),dtype=float)

    # State vector
    u_b_mps     = x[0]
    v_b_mps     = x[1]
    w_b_mps     = x[2]
    p_b_rps     = x[3]
    q_b_rps     = x[4]
    r_b_rps     = x[5]
    phi_rad     = x[6]
    theta_rad   = x[7]
    psi_rad     = x[8]
    p1_n_m      = x[9]
    p2_n_m      = x[10]
    p3_n_m      = x[11]

    if DO_PERTURBATION == True and t == PERTURBATION_TIME:
        u_b_mps += 10
        v_b_mps += 50
        w_b_mps += 20
        p_b_rps += np.deg2rad(15)
        q_b_rps += np.deg2rad(-12)
        r_b_rps += np.deg2rad(8)
        print("Doing perturbation")



    # Computing euler angles
    c_phi       = math.cos(phi_rad)
    c_theta     = math.cos(theta_rad)
    c_psi       = math.cos(psi_rad)
    s_phi       = math.sin(phi_rad)
    s_theta     = math.sin(theta_rad)
    s_psi       = math.sin(psi_rad)
    t_theta     = math.tan(theta_rad)

    # Mass and inertial elements
    m_kg        = vehicle_data.mass_kg
    Jxx_b_kgm2  = vehicle_data.Jxx
    Jyy_b_kgm2  = vehicle_data.Jyy
    Jzz_b_kgm2  = vehicle_data.Jzz
    Jxz_b_kgm2  = vehicle_data.Jxz


    # Altitude and Atmosphere model
    h_m = -p3_n_m

    rho_interp_kgm3         = np.interp(h_m, atmos_data["alt_m"], atmos_data["rho_kgpm3"])
    c_interp_mps            = np.interp(h_m, atmos_data["alt_m"], atmos_data["c_mps"])
    true_air_speed_mps      = math.sqrt(u_b_mps ** 2 + v_b_mps ** 2 + w_b_mps ** 2)
    dynamic_pressure_Npm2   = 0.5 * rho_interp_kgm3 * true_air_speed_mps ** 2
    mach                    = true_air_speed_mps / c_interp_mps


    # To avoid divide by 0
    if u_b_mps == 0 and w_b_mps == 0:
        w_over_u = 0
    else:
        w_over_u = w_b_mps / u_b_mps

    if v_b_mps == 0 and true_air_speed_mps == 0:
        v_over_vT = 0
    else:
        v_over_vT = v_b_mps / true_air_speed_mps

    # Angle of attack and side slip definitions
    alpha_rad   = math.atan(w_over_u)
    beta_rad    = math.asin(v_over_vT)

    s_alpha     = math.sin(alpha_rad)
    c_alpha     = math.cos(alpha_rad)
    s_beta      = math.sin(beta_rad)
    c_beta      = math.cos(beta_rad)

    gz_n_mps2 = np.interp(h_m, atmos_data["alt_m"], atmos_data["g_mps2"])

    # Gravity in body axis
    gx_b_mps2 = -s_theta * gz_n_mps2
    gy_b_mps2 = s_phi * c_theta * gz_n_mps2
    gz_b_mps2 = c_phi * c_theta * gz_n_mps2

    # Thrust force
    thrust_N = vehicle_data.find_total_thrust_N(rho_interp_kgm3, u_b_mps)
    thrust_N = 6100
    
    # Aerodynamic forces
    if type(vehicle_data) == Aircraft:
        drag_N = vehicle_data.find_total_drag_coeff(mach, np.rad2deg(alpha_rad)) * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2
        side_N = 0
        lift_N = vehicle_data.find_total_lift_coeff(mach, np.rad2deg(alpha_rad)) * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2

    elif type(vehicle_data) == VehicleData:
        drag_N = vehicle_data.cd_approx * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2
        side_N = 0
        lift_N = 0

    # Summation External forces
    Fx_b_N = - (c_alpha * c_beta * (drag_N - thrust_N) - c_alpha * s_beta * side_N - s_alpha * lift_N)
    Fy_b_N = -(s_beta * (drag_N - thrust_N) + c_beta * side_N)
    Fz_b_N = -(s_alpha * c_beta * (drag_N - thrust_N) - s_alpha * s_beta * side_N + c_alpha * lift_N)
    

    # Calculate moments
    M_moment_Nm = vehicle_data.find_total_pitch_moment_coeff(mach, np.rad2deg(alpha_rad)) * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2 * vehicle_data.mean_chord
    L_moment_Nm = vehicle_data.find_roll_coeff(mach, beta_rad) * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2 * vehicle_data.mean_chord
    N_moment_Nm = vehicle_data.find_yaw_coeff(mach, beta_rad) * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2 * vehicle_data.mean_chord

    # Calculate damping moments
    M_damping_Nm = vehicle_data.find_pitch_damping_coeff(mach, q_b_rps) * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2 * vehicle_data.mean_chord
    L_damping_Nm = vehicle_data.find_roll_damping_coeff(mach, p_b_rps)  * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2 * vehicle_data.mean_chord
    N_damping_Nm = vehicle_data.find_yaw_damping_coeff(mach, r_b_rps)  * dynamic_pressure_Npm2 * vehicle_data.a_ref_m2 * vehicle_data.mean_chord

    l_b_Nm = L_moment_Nm - L_damping_Nm
    m_b_Nm = M_moment_Nm + M_damping_Nm
    n_b_Nm = N_moment_Nm + N_damping_Nm
    
    # Denominator for roll and yaw
    denom_inv =  1 /((Jxx_b_kgm2 * Jzz_b_kgm2) - Jxz_b_kgm2 ** 2)

    # x-axis (roll) velocity
    dx[0] = (1 / m_kg) * Fx_b_N + gx_b_mps2 - w_b_mps * q_b_rps + v_b_mps * r_b_rps
    
    # y-axis (pitch) velocity
    dx[1] = (1 / m_kg) * Fy_b_N + gy_b_mps2 - u_b_mps * r_b_rps + w_b_mps * p_b_rps
    
    # z-axis (yaw) velocity
    dx[2] = (1 / m_kg) * Fz_b_N + gz_b_mps2 - v_b_mps * p_b_rps + u_b_mps * q_b_rps

    # roll equation
    dx[3] = (Jxz_b_kgm2 * (Jxx_b_kgm2 - Jyy_b_kgm2 + Jzz_b_kgm2) * p_b_rps * q_b_rps - \
            (Jzz_b_kgm2 * (Jzz_b_kgm2 - Jyy_b_kgm2) + Jxz_b_kgm2 ** 2) * q_b_rps * r_b_rps + \
            Jzz_b_kgm2 * l_b_Nm + \
            Jxz_b_kgm2 * n_b_Nm) * denom_inv
    
    # pitch equation
    dx[4] = ((Jzz_b_kgm2 - Jxx_b_kgm2) * p_b_rps * r_b_rps - \
            Jxz_b_kgm2 * (p_b_rps ** 2 - r_b_rps ** 2) + m_b_Nm) / Jyy_b_kgm2
    
    # yaw equation
    dx[5] = ((Jxx_b_kgm2 * (Jxx_b_kgm2 - Jyy_b_kgm2 )+ Jzz_b_kgm2 ** 2) * p_b_rps * q_b_rps + \
            Jxz_b_kgm2 * (Jxx_b_kgm2 - Jyy_b_kgm2 + Jzz_b_kgm2) * q_b_rps * r_b_rps + \
            Jxz_b_kgm2 * l_b_Nm + \
            Jxz_b_kgm2 * n_b_Nm) * denom_inv

    # kinematic equations
    dx[6] = p_b_rps +  s_phi * t_theta * q_b_rps + c_phi * t_theta * r_b_rps
    
    dx[7] =     c_phi * q_b_rps - s_phi * r_b_rps
    
    dx[8] =     s_phi / c_theta * q_b_rps + c_phi / c_theta * r_b_rps

    # position equations
    dx[9]   = c_theta * c_phi * u_b_mps + \
            (-c_phi * s_psi + s_phi * s_theta * c_psi) * v_b_mps + \
            (s_phi * s_psi + c_phi * s_theta * c_psi) * w_b_mps
    
    dx[10]   =  c_phi * s_psi * u_b_mps + \
            (c_phi * c_psi + s_phi * s_theta * s_psi) * v_b_mps + \
            (-s_phi * c_psi + c_phi * s_theta * s_psi) * w_b_mps
    
    dx[11]  = -s_theta * u_b_mps + \
            s_phi * c_theta * v_b_mps + \
            c_phi * c_theta * w_b_mps
    
    if DO_PERTURBATION == True and t == PERTURBATION_TIME:
        x_new = np.array([ 
                    u_b_mps,
                    v_b_mps,
                    w_b_mps,
                    p_b_rps,
                    q_b_rps,
                    r_b_rps,
                    dx[6],
                    dx[7],
                    dx[8],
                    dx[9],
                    dx[10],
                    dx[11],
                ])
        print(x_new)
        return x_new
    
    if PRINT_READ_OUTS == True:
        print(f"Drag: {drag_N:.1f}N\tLift: {lift_N:.1f}N\tThrust: {thrust_N:.1f}N\tMa: {mach:.3f}\tu_mps: {u_b_mps:.1f}m/s\tAoA: {np.rad2deg(alpha_rad):.3f}\tSideslip: {beta_rad:.3f}")
        print(f"Pitching Moment: {M_moment_Nm:.3f}\tCm: {vehicle_data.find_total_pitch_moment_coeff(mach, np.rad2deg(alpha_rad)):.3f}\tPitch Damp: {M_damping_Nm:.3f}\tFinal_pitch_M: {m_b_Nm:.3f}")
        print(f"Roll Moment: {L_moment_Nm:.3f}\tClb: {vehicle_data.find_roll_coeff(mach, beta_rad):.3f}\tRoll Damp: {L_damping_Nm:.3f}\tFinal_Roll_M: {l_b_Nm:.3f}")
        print(f"Yaw Moment: {N_moment_Nm:.3f}\tCnb: {vehicle_data.find_yaw_coeff(mach, beta_rad):.3f}\tYaw Damp: {N_damping_Nm:.3f}\tFinal_Yaw_M: {n_b_Nm:.3f}")
     

    return dx
