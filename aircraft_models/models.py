# ========================================================================
#   6 DOF simulator
#   Date: 8/09/2024
#   About:  This file contains the classes used to make a aircraft model
# 
#   Author: Oliver Clements
#       
# ========================================================================

# Libary imports
import math
import numpy as np

# Index for the aircraft dictionaries
MACH_INDEX  = 0
COEFF_INDEX = 1

# Angle of attack bounds (relevant only to mustang)
MAX_ALPHA = 6
MIN_ALPHA = -4


class VehicleData:
    def __init__(self,name) -> None:
        """
        General vehicle data class.
        
        :param name: Name of the vehicle/object.
        :param mass_kg: Mass of the vehicle/object in kilograms.
        :param cd_approx: Approximate drag coefficient.
        """

        self.name = name

        self.cd_approx = None
        self.mass_kg = None
        self.a_ref_m2 = None

        self.Jxx = None
        self.Jyy = None
        self.Jzz = None
        self.Jxz = None


    def __str__(self) -> str:
        return f"Name: {self.name}, Mass: {self.mass_kg} kg, Area: {self.a_ref_m2} m2"
    

class Aircraft(VehicleData):
    """
    Class for aircraft. Contains extra methods for drag, lift and moment coefficients
    
    """

    def __init__(self, name) -> None:
        # Include all other properties of the class
        super().__init__(name)

        # Drag and lift coeffs data
        self.cd0_data = None
        self.cl0_data = None
        self.cla_data = None

        # Moment coeffs data
        self.cm0_data = None
        self.cmq_data = None  # Pitch damping moment coefficient (function of Ma or alpha)
        self.cma_data = None
        self.clb_data = None
        self.clq_data = None
        self.cnb_data = None
        self.cnq_data = None

        self.aspect_ratio_m2 = None
        self.mean_chord = None
        self.oswald = None

        # Thrust properties
        self.prop_area_m2 = None
        self.prop_efficiency = None


    def __str__(self) -> str:
        return f"\tName: {self.name}, Mass: {self.mass_kg} kg, Wing Area: {self.a_ref_m2} m2 \n \
                Aspect Ratio: {self.aspect_ratio_m2}, Oswald efficiency: {self.oswald}    \n \
                Jxx: {self.Jxx}m4, Jyy: {self.Jyy}m4, Jzz: {self.Jzz}m4, Jxz: {self.Jxz}m3"


    def create_aero_dict(self, file_name: str, aero_type: str) -> None:
        """ Creates the aerodynamic dict"""
        data = np.loadtxt(file_name, delimiter=',', skiprows=1)

        if hasattr(self, aero_type):
            setattr(self, aero_type, data.T)

        else:
            raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{aero_type}'")
        
        
    def find_coefficient(self, Ma: float, aero_type: str) -> float:
        """ Finds a aerodynamic coefficient for a given mach number"""
        
        if hasattr(self, aero_type):
            aero_data = getattr(self, aero_type)

            return np.interp(Ma, aero_data[MACH_INDEX], aero_data[COEFF_INDEX])

        else:
            raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{aero_type}'")
        

    def find_total_lift_coeff(self, Ma: float, alpha: float) -> float:
        """ Finds the total lift coefficient for a given Ma"""

        C_l0 = self.find_coefficient(Ma, "cl0_data")
        C_la = self.find_coefficient(Ma, "cla_data")

        if alpha >= MAX_ALPHA or alpha <= MIN_ALPHA:
            print("Warning: Angle of attack outside valid rang")


        return C_l0 + C_la * alpha
    
    
    def find_total_drag_coeff(self, Ma: float, alpha: float) -> float:
        """ Finds the total lift coefficient for a given Ma"""

        C_d0 = self.find_coefficient(Ma, "cd0_data")
        C_l = self.find_total_lift_coeff(Ma, alpha)
        
        return C_d0 + (C_l ** 2 / (math.pi * self.oswald * self.aspect_ratio_m2))
    
    
    def find_total_thrust_N(self, rho_kgpm3: float, u_mps) -> float:
        """ Finds the total thrust for a prop plane"""
        # velocity of air through prop
        v0 = 3 * u_mps / 2  # This is assuming the prop can adjust pitch and keep 2 * v_in = v_out
   
        return 2 * rho_kgpm3 * self.prop_area_m2 * self.prop_efficiency * v0 ** 2
    
    def find_total_pitch_moment_coeff(self, Ma: float, alpha: float) -> float:
        """ Finds the pitching moment coefficient for a given Cl"""

        C_m0 = self.find_coefficient(Ma, "cm0_data")
        C_ma = self.find_coefficient(Ma, 'cma_data')

        return C_m0 + C_ma * alpha
    
    def find_pitch_damping_coeff(self, Ma: float, q_b_rps: float) -> float:
        """ Finds the total pitch damping coefficient"""
        return self.find_coefficient(Ma, "cmq_data") * q_b_rps
    

    def find_roll_coeff(self, Ma: float, beta: float) -> float:
        """ Finds the total roll coefficient with respect to sideslip"""
        C_lb = self.find_coefficient(Ma, 'clb_data')
        return C_lb * beta
    

    def find_roll_damping_coeff(self, Ma: float, p_b_dps: float) -> float:
        """ Finds the total roll damping coefficient"""
        C_lq = self.find_coefficient(Ma, 'clq_data')
        return C_lq * p_b_dps
    

    def find_yaw_coeff(self, Ma: float, beta: float) -> float:
        """ Finds the total yaw coefficient with respect to sideslip"""
        C_nb = self.find_coefficient(Ma, 'cnb_data')
        return C_nb * beta
    

    def find_yaw_damping_coeff(self, Ma: float, r_b_dps: float) -> float:
        """ Finds the total roll damping coefficient"""
        C_nq = self.find_coefficient(Ma, 'cnq_data')
        return C_nq * r_b_dps

        

def get_Mustang() -> Aircraft:
    """ Returns all the class property of the mustang"""

    aircraft_name = "Mustang"
    mustang = Aircraft(aircraft_name)

    # Mass and area properties
    mustang.mass_kg = 4445
    mustang.a_ref_m2 = 21.65
    mustang.aspect_ratio_m2 = 5.9
    mustang.mean_chord = 0.001
    # mustang.mean_chord = 1

    # Second Moments of inertia
    mustang.Jxx = 69.09
    mustang.Jyy = 79.67
    mustang.Jzz = 125.1
    mustang.Jxz = -10    # Big estimation using Jxx - Jyy

    mustang.oswald = 0.9

    mustang.prop_area_m2 = 9.08
    mustang.prop_efficiency = 0.006


    # File names
    try:
        cd0_file_name = "./Mustang_aero/Mustang_CD0.csv"
        cl0_file_name = "./Mustang_aero/Mustang_CL0.csv"
        cla_file_name = "./Mustang_aero/Mustang_CLa.csv"
        cm0_file_name = "./Mustang_aero/Mustang_Cm0.csv"
        cmq_file_name = "./Mustang_aero/Mustang_Cmq.csv"
        cma_file_name = "./Mustang_aero/Mustang_Cma.csv"
        clb_file_name = "./Mustang_aero/Mustang_Clb.csv"
        clq_file_name = "./Mustang_aero/Mustang_Clq.csv"
        cnb_file_name = "./Mustang_aero/Mustang_Cnb.csv"
        cnq_file_name = "./Mustang_aero/Mustang_Cnq.csv"

        # Open the aero data and store it in class
        mustang.create_aero_dict(cd0_file_name, 'cd0_data')
        mustang.create_aero_dict(cl0_file_name, 'cl0_data')
        mustang.create_aero_dict(cla_file_name, 'cla_data')
        mustang.create_aero_dict(cm0_file_name, 'cm0_data')
        mustang.create_aero_dict(cmq_file_name, 'cmq_data')
        mustang.create_aero_dict(cma_file_name, 'cma_data')
        mustang.create_aero_dict(clb_file_name, 'clb_data')
        mustang.create_aero_dict(clq_file_name, 'clq_data')
        mustang.create_aero_dict(cnb_file_name, 'cnb_data')
        mustang.create_aero_dict(cnq_file_name, 'cnq_data')

    except FileNotFoundError:
        cd0_file_name = "aircraft_models/Mustang_aero/Mustang_CD0.csv"
        cl0_file_name = "aircraft_models/Mustang_aero/Mustang_CL0.csv"
        cla_file_name = "aircraft_models/Mustang_aero/Mustang_CLa.csv"
        cm0_file_name = "aircraft_models/Mustang_aero/Mustang_Cm0.csv"
        cmq_file_name = "aircraft_models/Mustang_aero/Mustang_Cmq.csv"
        cma_file_name = "aircraft_models/Mustang_aero/Mustang_Cma.csv"
        clb_file_name = "aircraft_models/Mustang_aero/Mustang_Clb.csv"
        clq_file_name = "aircraft_models/Mustang_aero/Mustang_Clq.csv"
        cnb_file_name = "aircraft_models/Mustang_aero/Mustang_Cnb.csv"
        cnq_file_name = "aircraft_models/Mustang_aero/Mustang_Cnq.csv"

        # Open the aero data and store it in class
        mustang.create_aero_dict(cd0_file_name, 'cd0_data')
        mustang.create_aero_dict(cl0_file_name, 'cl0_data')
        mustang.create_aero_dict(cla_file_name, 'cla_data')
        mustang.create_aero_dict(cm0_file_name, 'cm0_data')
        mustang.create_aero_dict(cmq_file_name, 'cmq_data')
        mustang.create_aero_dict(cma_file_name, 'cma_data')
        mustang.create_aero_dict(clb_file_name, 'clb_data')
        mustang.create_aero_dict(clq_file_name, 'clq_data')
        mustang.create_aero_dict(cnb_file_name, 'cnb_data')
        mustang.create_aero_dict(cnq_file_name, 'cnq_data')

    
    print(mustang)

    return mustang



def get_Bowlingball() -> VehicleData:
    """ Returns the vehicle data for a sphere"""
    ball_name = "Bowling ball"
    
    bowlingball = VehicleData(name=ball_name)

    r_sphere_m = 0.1
    m_sphere_kg = 5
    Cd_approx = 0.5

    A_ref_m2 = math.pi * r_sphere_m ** 2
    J_sphere_kgm2 = 0.4 * m_sphere_kg * r_sphere_m ** 2

    bowlingball.mass_kg = m_sphere_kg
    bowlingball.cd_approx = Cd_approx
    bowlingball.a_ref_m2 = A_ref_m2
    bowlingball.Jxx = J_sphere_kgm2
    bowlingball.Jyy = J_sphere_kgm2
    bowlingball.Jzz = J_sphere_kgm2
    bowlingball.Jxz = 0
    

    return bowlingball

