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
    def make_params(time, V_inf, rho_inf, c_len, area, inertia, angle_command, motor_params):
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
            'time': time,
            'V_inf': V_inf,
            'rho_inf': rho_inf,
            'c_len': c_len,
            'area': area,
            'inertia': inertia,
            'angle_command': angle_command,
            'motor_params': motor_params
        }

    @staticmethod
    def make_state_info(time, r, c, V_rel, V, C_l, C_d, T, L, D, E, R, F, aoa, power, alpha, omega, motor_state_info):
        """
        Create an empty state information dictionary.
        
        Returns:
            dict: empty state information dictionary
        """
        return {
            'time': time,
            'r': r,
            'c': c,
            'V_rel': V_rel,
            'V': V,
            'C_l': C_l,
            'C_d': C_d,
            'T': T,
            'L': L,
            'D': D,
            'E': E,
            'R': R,
            'F': F,
            'aoa': aoa,
            'power': power,
            'alpha': alpha,
            'omega': omega,
            'motor_state_info': motor_state_info
        }

    @staticmethod
    def step(state, dt, params, simulate_motor=True):
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
        def aero_from_theta_omega(theta, omega_z, angle_from_r_local):
            # Geometry
            r_vec = state['B'] - state['A']
            radius = np.linalg.norm(r_vec)
            r = np.array([radius * np.cos(theta), radius * np.sin(theta), 0.0])
            # Kinematics
            omega_vec = np.array([0.0, 0.0, omega_z])
            V = - np.cross(omega_vec, r)
            V_rel = V + params['V_inf']
            # Avoid zero div
            if np.linalg.norm(V_rel) == 0.0:
                V_rel_safe = np.array([1e-8, 1e-8, 1e-8])
            else:
                V_rel_safe = V_rel
            # Dynamic pressure
            q_inf = 0.5 * params['rho_inf'] * np.linalg.norm(V_rel_safe) ** 2
            # Chord and AoA
            perpendicular_to_r = WindTurbineBlade.calc_rotate_vector(r, 90 * np.pi / 180)
            perpendicular_to_r = perpendicular_to_r / np.linalg.norm(perpendicular_to_r) * params['c_len']
            c = WindTurbineBlade.calc_rotate_vector(perpendicular_to_r, angle_from_r_local)
            aoa = WindTurbineBlade.calc_angle_between_vectors(V_rel_safe, -c)
            if aoa > np.pi / 2:
                aoa = np.pi - aoa
            # Coefficients
            C_l = WindTurbineBlade.get_lift_coefficient(aoa)
            C_d = WindTurbineBlade.get_drag_coefficient(aoa)
            # Forces
            lift_direction = np.array([
                np.cos(-90 * np.pi / 180) * V_rel_safe[0] - np.sin(-90 * np.pi / 180) * V_rel_safe[1],
                np.sin(-90 * np.pi / 180) * V_rel_safe[0] + np.cos(-90 * np.pi / 180) * V_rel_safe[1], 
                0.0
            ])
            L = C_l * q_inf * params['area'] * (lift_direction / np.linalg.norm(lift_direction))
            D = C_d * q_inf * params['area'] * (V_rel_safe / np.linalg.norm(V_rel_safe))
            E = L + D
            R = np.dot(E, r) / (np.linalg.norm(r) ** 2) * r
            F = E - R
            T = np.cross(r, F)
            alpha_vec = T / params['inertia']
            return {
                'r': r,
                'c': c,
                'V_rel': V_rel,
                'V': V,
                'C_l': C_l,
                'C_d': C_d,
                'T': T,
                'L': L,
                'D': D,
                'E': E,
                'R': R,
                'F': F,
                'aoa': aoa,
                'alpha_vec': alpha_vec,
            }

        def f(theta, omega_z, angle_from_r_local):
            aero = aero_from_theta_omega(theta, omega_z, angle_from_r_local)
            theta_dot = omega_z
            omega_dot = aero['alpha_vec'][2]
            return theta_dot, omega_dot, aero

        # Motor update (independent integrator handled in MotorModel)
        motor_state = state['motor_state']
        motor_params = params['motor_params']
        motor_params['target_theta'] = params['angle_command']
        new_motor_state, motor_state_info = MotorModel.step(motor_state, motor_params, dt)
        angle_from_r = motor_state_info['theta']
        if not simulate_motor:
            new_motor_state = state['motor_state']
            angle_from_r = params['angle_command']

        # Current generalized coordinates
        r_current = state['B'] - state['A']
        theta0 = np.arctan2(r_current[1], r_current[0])
        omega0 = state['omega'][2]

        # Integrate blade dynamics
        integrator = params.get('integrator', 'euler')
        if integrator == 'rk4':
            k1_theta, k1_omega, aero1 = f(theta0, omega0, angle_from_r)
            k2_theta, k2_omega, _ = f(theta0 + 0.5 * dt * k1_theta, omega0 + 0.5 * dt * k1_omega, angle_from_r)
            k3_theta, k3_omega, _ = f(theta0 + 0.5 * dt * k2_theta, omega0 + 0.5 * dt * k2_omega, angle_from_r)
            k4_theta, k4_omega, aero4 = f(theta0 + dt * k3_theta, omega0 + dt * k3_omega, angle_from_r)
            theta_new = theta0 + (dt / 6.0) * (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta)
            omega_new = omega0 + (dt / 6.0) * (k1_omega + 2 * k2_omega + 2 * k3_omega + k4_omega)
            # Use final-stage aero for reporting
            aero_final = aero4
            alpha_vec = aero_final['alpha_vec']
            T = aero_final['T']
            r = aero_final['r']
            c = aero_final['c']
            V_rel = aero_final['V_rel']
            V = aero_final['V']
            C_l = aero_final['C_l']
            C_d = aero_final['C_d']
            L = aero_final['L']
            D = aero_final['D']
            E = aero_final['E']
            R = aero_final['R']
            F = aero_final['F']
            aoa = aero_final['aoa']
        else:
            k1_theta, k1_omega, aero1 = f(theta0, omega0, angle_from_r)
            theta_new = theta0 + dt * k1_theta
            omega_new = omega0 + dt * k1_omega
            alpha_vec = aero1['alpha_vec']
            T = aero1['T']
            r = aero1['r']
            c = aero1['c']
            V_rel = aero1['V_rel']
            V = aero1['V']
            C_l = aero1['C_l']
            C_d = aero1['C_d']
            L = aero1['L']
            D = aero1['D']
            E = aero1['E']
            R = aero1['R']
            F = aero1['F']
            aoa = aero1['aoa']

        # Construct new state
        radius = np.linalg.norm(state['B'] - state['A'])
        r_new = np.array([radius * np.cos(theta_new), radius * np.sin(theta_new), 0.0])
        B_new = state['A'] + r_new
        new_omega = state['omega'].copy()
        new_omega[2] = omega_new
        new_state = WindTurbineBlade.make_state(state['A'], B_new, new_omega, new_motor_state)
        power = new_omega[2] * T[2]

        # Update state information
        state_information = WindTurbineBlade.make_state_info(
            params['time'], r, c, V_rel, V, C_l, C_d, T[2], L, D, E, R, F, aoa, power, alpha_vec, new_omega[2], motor_state_info
        )
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