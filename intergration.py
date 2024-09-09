# ========================================================================
#   6 DOF simulator
#   Date: 8/09/2024
#   About:  This file contains the the numerical integration methods
#           (currently only forward euler)
# 
#   Author: Oliver Clements
#           Based upon Ben Dicksons Youtube series
#           Link: https://www.youtube.com/watch?v=hr_PqdkG6XY&list=PLcmbTy9X3gXs4JVXYucrMz5bJ4ZuaEGJ_&ab_channel=BenDickinson
# ========================================================================

# Library Imports
import numpy as np
from typing import Union

# Module Imports
from aircraft_models.models import VehicleData, Aircraft

def foward_euler(func, t_s, x, h_s, vehicle_data: Union[VehicleData, Aircraft], atmos_data):
    """ Numerical integration using the forward euler method
    
    Augments:
        func:   function f(t,x) = dx/dt
        t_s:    time vector:
        x:      numerical approximated solution
        h_s:    time step
        
    Returns: t_s, x
    
    """
    for i in range(1, len(t_s)):
        x[:,i] = x[:, i-1] + h_s * func(t_s[i-1], x[:,i-1], vehicle_data, atmos_data)

    return t_s, x
