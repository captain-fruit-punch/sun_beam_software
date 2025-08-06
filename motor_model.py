import numpy as np

class MotorModel:
    def make_state(theta, omega, error_i):
        new_state = {
            'theta': theta,
            'omega': omega,
            'error_i': error_i
        }
        return new_state
    
    def make_perams(target_theta, torque_speed, inertia, input_torque, P, D, I):
        new_perams = {
            'target_theta': target_theta,
            'torque_speed': torque_speed,
            'inertia': inertia,
            'input_torque': input_torque,
            'P': P,
            'D': D,
            'I': I
        }
        
    def make_state_info(theta, omega, alpha, error_p, error_v, error_i, tau_out, target_omega):
        new_state_info = {
            'theta': theta,
            'omega': omega,
            'alpha': alpha,
            'error_p': error_p,
            'error_v': error_v,
            'tau_out': tau_out,
            'target_omega': target_omega,
            'error_i': error_i
        }
        return new_state_info
    
    def step(state, perams, dt):
        error_p = perams['target_theta'] - state['theta']
        target_omega = perams['P'] * error_p
        error_v = target_omega - state['omega']
        error_i = error_i + error_v * dt
        tau_out = perams['I'] * error_i + perams['D'] * error_v
        alpha = tau_out / perams['inertia']
        omega = state['omega'] + alpha * dt
        theta = state[theta] + omega * dt
        new_state_info = MotorModel.make_state_info(theta, omega, alpha, error_p, error_v, error_i, tau_out, target_omega)
        new_state = MotorModel.make_state(theta, omega, error_i)
        
        return new_state, new_state_info