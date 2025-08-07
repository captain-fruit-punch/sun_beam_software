import numpy as np
import pandas as pd
from motor_model import MotorModel

class WindTurbineBlade:
    @staticmethod
    def calc_angle_between_vectors(v, w):
        """Calculate the angle between two 2D vectors."""
        angle = np.arctan2(v[0] * w[1] - v[1] * w[0], v[0] * w[0] + v[1] * w[1])
        return angle

    @staticmethod
    def calc_rotate_vector(v, angle):
        """Rotate a 2D vector by a given angle."""
        return np.array([
            np.cos(angle) * v[0] - np.sin(angle) * v[1],
            np.sin(angle) * v[0] + np.cos(angle) * v[1], 
            0.0
        ])

    @staticmethod
    def get_lift_coefficient(alpha):
        """
        Get lift coefficient by linearly interpolating data from cl_vs_alpha.csv.
        For negative alpha values, returns the same result as positive alpha (symmetric).
        """
        # Load the data
        data = pd.read_csv('cl_vs_alpha.csv')
        alphas = data['alpha'].values
        cls = data['cl'].values
        
        # Use absolute value of alpha for symmetric behavior
        alpha_abs = np.abs(alpha)
        
        # Interpolate using numpy
        cl = abs(alpha) * 2 * np.pi
        if abs(alpha) > 7 * np.pi / 180:
            cl = float(np.interp(alpha_abs, alphas, cls))
        cl = cl * np.sign(alpha)
        return cl

    @staticmethod
    def get_drag_coefficient(alpha):
        """
        Get drag coefficient by linearly interpolating data from cd_vs_alpha.csv.
        For negative alpha values, returns the same result as positive alpha (symmetric).
        """
        # Load the data
        data = pd.read_csv('cd_vs_alpha_corrected.csv')
        alphas = data['alpha'].values
        cds = data['cd'].values
        
        # Use absolute value of alpha for symmetric behavior
        alpha_abs = np.abs(alpha)
        
        # Interpolate using numpy
        cd = float(np.interp(alpha_abs, alphas, cds))
        return cd

    @staticmethod
    def make_state(A, B, omega, motor_state):
        """
        Create a state dictionary for the wind turbine blade.
        
        Args:
            A: pivot point position (numpy array)
            B: blade tip position (numpy array)
            omega: angular velocity vector (numpy array)
            motor_state: motor state dictionary
        
        Returns:
            dict: state dictionary
        """
        return {
            'A': A,
            'B': B,
            'omega': omega,
            'motor_state': motor_state
        }

    @staticmethod
    def make_params(V_inf, rho_inf, c_len, area, inertia, angle_command, motor_params):
        """
        Create a parameters dictionary for the wind turbine blade simulation.
        
        Args:
            V_inf: freestream velocity (numpy array)
            rho_inf: air density (float)
            c_len: chord length (float)
            area: blade area (float)
            inertia: moment of inertia (numpy array)
            angle_command: angle of attack command (float, radians)
            motor_params: motor parameters dictionary
        
        Returns:
            dict: parameters dictionary
        """
        return {
            'V_inf': V_inf,
            'rho_inf': rho_inf,
            'c_len': c_len,
            'area': area,
            'inertia': inertia,
            'angle_command': angle_command,
            'motor_params': motor_params
        }

    @staticmethod
    def make_state_info():
        """
        Create an empty state information dictionary.
        
        Returns:
            dict: empty state information dictionary
        """
        return {
            'r': np.zeros((1,3)),
            'c': np.zeros((1,3)),
            'V_rel': np.zeros((1,3)),
            'V': np.zeros((1,3)),
            'C_l': 0.0,
            'C_d': 0.0,
            'T': np.zeros((1,3)),
            'L': np.zeros((1,3)),
            'D': np.zeros((1,3)),
            'E': np.zeros((1,3)),
            'R': np.zeros((1,3)),
            'F': np.zeros((1,3)),
            'aoa': 0.0,
            'power': 0.0,
            'alpha': 0.0,
            'omega': 0.0,
            'motor_state_info': {}
        }

    @staticmethod
    def step(state, dt, params):
        """
        Single simulation step that takes current state and returns next state.
        
        Args:
            state: dict containing current state variables
                - A: pivot point position
                - B: blade tip position  
                - omega: angular velocity vector
                - motor_state: motor state
            dt: time step
            params: dict containing simulation parameters
                - V_inf: freestream velocity
                - rho_inf: air density
                - c_len: chord length
                - area: blade area
                - inertia: moment of inertia
                - angle_command: angle of attack command
                - motor_params: motor parameters
        
        Returns:
            tuple: (new_state, state_information)
        """
        # Extract state variables
        A = state['A']
        B = state['B'] 
        omega = state['omega']
        
        # Extract parameters
        V_inf = params['V_inf']
        rho_inf = params['rho_inf']
        c_len = params['c_len']
        area = params['area']
        inertia = params['inertia']
        angle_command = params['angle_command']
        
        # Initialize state information
        state_information = WindTurbineBlade.make_state_info()
        
        # Calculate relative position vector
        r = B - A
        
        # Calculate velocity at blade tip
        V = - np.cross(omega, r)
        V_rel = V + V_inf
        
        # Motor model
        motor_state = state['motor_state']
        motor_params = params['motor_params']
        motor_params['target_theta'] = angle_command
        new_motor_state, motor_state_info = MotorModel.step(motor_state, motor_params, dt)

        # Initialize variables
        new_omega = omega.copy()
        alpha = np.zeros(3)
        C_l = 0.0
        C_d = 0.0
        L = np.zeros(3)
        D = np.zeros(3)
        E = np.zeros(3)
        R = np.zeros(3)
        F = np.zeros(3)
        T = np.zeros(3)
        c = np.zeros(3)
        aoa = 0.0

        if np.linalg.norm(V_rel) != 0.0:
            # Calculate dynamic pressure
            q_inf = 0.5 * rho_inf * np.linalg.norm(V_rel) ** 2
            
            # Calculate optimal angle command
            perpendicular_to_r = WindTurbineBlade.calc_rotate_vector(r, 90 * np.pi / 180)
            perpendicular_to_r = perpendicular_to_r / np.linalg.norm(perpendicular_to_r) * c_len
            
            # Calculate chord vector
            c = perpendicular_to_r
            angle_from_r = motor_state_info['theta']
            c = WindTurbineBlade.calc_rotate_vector(c, angle_from_r)
            
            # Calculate angle of attack
            aoa = WindTurbineBlade.calc_angle_between_vectors(V_rel, -c)
            if aoa > np.pi / 2:
                aoa = np.pi - aoa
            
            # Get aerodynamic coefficients
            C_l = WindTurbineBlade.get_lift_coefficient(aoa)
            C_d = WindTurbineBlade.get_drag_coefficient(aoa)
            
            # Calculate forces
            lift_direction = np.array([
                np.cos(-90 * np.pi / 180) * V_rel[0] - np.sin(-90 * np.pi / 180) * V_rel[1],
                np.sin(-90 * np.pi / 180) * V_rel[0] + np.cos(-90 * np.pi / 180) * V_rel[1], 
                0.0
            ])
            L = C_l * q_inf * area * (lift_direction / np.linalg.norm(lift_direction))  # lift force
            D = C_d * q_inf * area * (V_rel / np.linalg.norm(V_rel))  # drag force
            E = L + D  # total force
            
            # Calculate reaction and resultant forces
            R = np.dot(E, r) / (np.linalg.norm(r) ** 2) * r  # reaction force
            F = E - R  # resultant force
            
            # Calculate torque and angular acceleration
            T = np.cross(r, F)  # torque
            alpha = T / inertia  # angular acceleration
            
            # Update angular velocity
            omega_rotation = omega[2]
            alpha_rotation = alpha[2]
            new_omega[2] = omega_rotation + alpha_rotation * dt
        
        # Update angular position (simplified rotation)
        theta = new_omega * dt
        r_new = WindTurbineBlade.calc_rotate_vector(r, theta[2])
        B_new = A + r_new
        
        new_state = WindTurbineBlade.make_state(A, B_new, new_omega, new_motor_state)
        
        # Update state information
        state_information['alpha'] = alpha[2]
        state_information['omega'] = new_omega[2]
        state_information['r'] = r
        state_information['c'] = c
        state_information['V_rel'] = V_rel
        state_information['V'] = V
        state_information['C_l'] = C_l
        state_information['C_d'] = C_d
        state_information['L'] = L
        state_information['D'] = D
        state_information['E'] = E
        state_information['R'] = R
        state_information['F'] = F
        state_information['T'] = T
        state_information['aoa'] = aoa
        state_information['power'] = omega[2] * T[2]
        state_information['motor_state_info'] = motor_state_info
        
        return new_state, state_information

    @staticmethod
    def copy_state_info_to_array(state_info, state_info_array):
        """
        Copy state information to an array for plotting purposes.
        
        Args:
            state_info: current state information dictionary
            state_info_array: existing state information array
            
        Returns:
            dict: updated state information array
        """
        if state_info_array == {}:
            new_state_info_array = {}
            for key in state_info.keys():
                if key != 'motor_state_info':  # Skip nested motor state info
                    new_state_info_array[key] = [state_info[key]]
            return new_state_info_array
        
        new_state_info_array = {}
        for key in state_info.keys():
            if key != 'motor_state_info':  # Skip nested motor state info
                new_state_info_array[key] = state_info_array[key] + [state_info[key]]
        
        return new_state_info_array 