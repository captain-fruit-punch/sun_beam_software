import numpy as np
import copy
class MotorModel:
    def make_state(theta, omega, error_i):
        new_state = {
            'theta': theta,
            'omega': omega,
            'error_i': error_i
        }
        return new_state
    
    def make_params(time, target_theta, torque_speed, inertia, input_torque, P, D, I, c=0):
        new_params = {
            'time': time,
            'target_theta': target_theta,
            'torque_speed': torque_speed,
            'inertia': inertia,
            'input_torque': input_torque,
            'P': P,
            'D': D,
            'I': I,
            'c': c
        }
        return new_params
        
    def make_state_info(time, theta, omega, alpha, error_p, error_v, error_i, tau_out, target_omega):
        new_state_info = {
            'time': time,
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
    
    def copy_state_info_to_array(state_info, state_info_array):
        if state_info_array == {}:
            new_state_info_array = {}
            for key in state_info.keys():
                new_state_info_array[key] = [state_info[key]]
            return new_state_info_array
        
        new_state_info_array = {}
        for key in state_info.keys():
            new_state_info_array[key] = state_info_array[key] + [state_info[key]]
        
        return new_state_info_array
    
    def make_motor_torque_speed(max_speed, max_torque):
        """
        Creates a motor torque-speed function that takes omega and tau as inputs
        and returns output_tau with the following constraints:
        - Maximum bounds follow slope of max_torque/max_speed with intercept at max_torque
        - Absolute maximum is max_torque
        """
        # Calculate the slope for the torque-speed characteristic
        slope = max_torque / max_speed
        
        def motor_torque_speed(omega, tau):
            """
            Motor torque-speed function
            
            Args:
                omega: angular velocity (rad/s)
                tau: input torque (N⋅m)
            
            Returns:
                output_tau: output torque (N⋅m) constrained by motor limits
            """
            # Calculate the maximum available torque at current speed
            # This follows the line: tau_max = max_torque - slope * omega
            # When omega = 0, tau_max = max_torque
            # When omega = max_speed, tau_max = 0
            max_available_torque = max_torque - slope * abs(omega)
            # print(f"max_available_torque: {max_available_torque}")
            # print(f"tau: {tau}")
            output_tau = min(max_available_torque, abs(tau))
            # print(f"output_tau: {output_tau}")
            # if abs(tau) < 0 and output_tau > 0:
            #     print(f"tau: {tau} is negative and output_tau: {output_tau} is positive")
            return output_tau * np.sign(tau)
        
        return motor_torque_speed
    
    def _derivatives(state, params):
        """Compute time-derivatives for motor state for ODE integration.

        Args:
            state: dict with keys 'theta', 'omega', 'error_i'
            params: dict with keys 'target_theta', 'torque_speed', 'inertia', 'P', 'D', 'I'

        Returns:
            derivs: dict with keys 'theta_dot', 'omega_dot', 'error_i_dot'
            aux: dict with useful intermediate values like error_p, error_v, tau_out, alpha, target_omega
        """
        error_p = params['target_theta'] - state['theta']
        target_omega = params['P'] * error_p
        error_v = target_omega - state['omega']
        # Control signal before motor limits
        control_tau = params['I'] * state['error_i'] + params['D'] * error_v
        # viscous damping
        damping_tau = params['c'] * state['omega'] * -1
        # Apply torque-speed saturation/limits
        tau_out = params['torque_speed'](state['omega'], control_tau + damping_tau) + damping_tau
        alpha = tau_out / params['inertia']
        theta_dot = state['omega']
        omega_dot = alpha
        error_i_dot = error_v
        derivs = {
            'theta_dot': theta_dot,
            'omega_dot': omega_dot,
            'error_i_dot': error_i_dot,
        }
        aux = {
            'error_p': error_p,
            'error_v': error_v,
            'tau_out': tau_out,
            'alpha': alpha,
            'target_omega': target_omega,
        }
        return derivs, aux

    def _step_euler(state, params, dt):
        derivs, aux = MotorModel._derivatives(state, params)
        theta = state['theta'] + derivs['theta_dot'] * dt
        omega = state['omega'] + derivs['omega_dot'] * dt
        error_i = state['error_i'] + derivs['error_i_dot'] * dt
        new_state = MotorModel.make_state(theta, omega, error_i)
        # Recompute aux for new state to report consistent info at end of step
        _, aux_final = MotorModel._derivatives(new_state, params)
        new_state_info = MotorModel.make_state_info(
            params['time'], theta, omega, aux_final['alpha'], aux_final['error_p'], aux_final['error_v'], error_i, aux_final['tau_out'], aux_final['target_omega']
        )
        return new_state, new_state_info

    def _step_rk4(state, params, dt):
        # k1
        k1, aux1 = MotorModel._derivatives(state, params)
        s2 = {
            'theta': state['theta'] + 0.5 * dt * k1['theta_dot'],
            'omega': state['omega'] + 0.5 * dt * k1['omega_dot'],
            'error_i': state['error_i'] + 0.5 * dt * k1['error_i_dot'],
        }
        # k2
        k2, _ = MotorModel._derivatives(s2, params)
        s3 = {
            'theta': state['theta'] + 0.5 * dt * k2['theta_dot'],
            'omega': state['omega'] + 0.5 * dt * k2['omega_dot'],
            'error_i': state['error_i'] + 0.5 * dt * k2['error_i_dot'],
        }
        # k3
        k3, _ = MotorModel._derivatives(s3, params)
        s4 = {
            'theta': state['theta'] + dt * k3['theta_dot'],
            'omega': state['omega'] + dt * k3['omega_dot'],
            'error_i': state['error_i'] + dt * k3['error_i_dot'],
        }
        # k4
        k4, _ = MotorModel._derivatives(s4, params)

        theta = state['theta'] + (dt / 6.0) * (
            k1['theta_dot'] + 2 * k2['theta_dot'] + 2 * k3['theta_dot'] + k4['theta_dot']
        )
        omega = state['omega'] + (dt / 6.0) * (
            k1['omega_dot'] + 2 * k2['omega_dot'] + 2 * k3['omega_dot'] + k4['omega_dot']
        )
        error_i = state['error_i'] + (dt / 6.0) * (
            k1['error_i_dot'] + 2 * k2['error_i_dot'] + 2 * k3['error_i_dot'] + k4['error_i_dot']
        )

        new_state = MotorModel.make_state(theta, omega, error_i)
        # Compute info with final state
        _, aux_final = MotorModel._derivatives(new_state, params)
        new_state_info = MotorModel.make_state_info(
            params['time'], theta, omega, aux_final['alpha'], aux_final['error_p'], aux_final['error_v'], error_i, aux_final['tau_out'], aux_final['target_omega']
        )
        return new_state, new_state_info

    def step(state, params, dt):
        integrator = params.get('integrator', 'euler')
        if integrator == 'rk4':
            return MotorModel._step_rk4(state, params, dt)
        else:
            return MotorModel._step_euler(state, params, dt)
    
    
def test_motor_model(params, state, time_step, simulation_length):
    num_steps = int(simulation_length / time_step)
    cur_time = 0
    state_info_array = {}
    for i in range(num_steps):
        params['time'] = cur_time
        new_state, new_state_info = MotorModel.step(state, params, time_step)
        new_state_info_array = MotorModel.copy_state_info_to_array(new_state_info, state_info_array)
        state = new_state
        state_info_array = new_state_info_array
        cur_time += time_step
    
    return state_info_array

def evaluate_settling_time(state_info_array, target_theta):
    error_p = state_info_array['error_p']
    if abs(error_p[-1]) > 0.05 * target_theta:
        return -1
    last_velocity = state_info_array['omega'][-1]
    if abs(last_velocity) > 0.05:
        return -1
    for i in range(len(error_p) - 1, 0, -1):
        if abs(error_p[i]) > 0.05 * target_theta:
            return state_info_array['time'][i]
    return -1

def evaluate_overshoot(state_info_array, target_theta):
    max_theta = max(state_info_array['theta'])
    overshoot = max_theta / target_theta
    return overshoot

def optimize_pid_params(P_range, D_range, I_range, params, state, time_step, simulation_length):
    best_state_info_array = {}
    best_params = params
    best_settling_time = np.inf
    for P in P_range:
        for D in D_range:
            for I in I_range:
                test_params = copy.deepcopy(params)
                test_params['P'] = P
                test_params['D'] = D
                test_params['I'] = I
                state_info_array = test_motor_model(test_params, state, time_step, simulation_length)
                time_to_settle = evaluate_settling_time(state_info_array, params['target_theta'])
                overshoot = evaluate_overshoot(state_info_array, params['target_theta'])
                # print(f"Overshoot: {overshoot}")
                # print(f"Test params: P={P}, D={D}, I={I}, time={time}, final_error={final_error}", end = " - ")
                if time_to_settle < best_settling_time and time_to_settle > 0 and overshoot < 1.1:
                    best_settling_time = time_to_settle
                    best_params = test_params
                    best_state_info_array = state_info_array
                    # print(f"New best params: P={P}, D={D}, I={I}, time={time}, final_error={final_error}")
                        
    print(f"Best params: P={best_params['P']}, D={best_params['D']}, I={best_params['I']}, time_to_settle={best_settling_time}")
    return best_params, best_state_info_array


def dual_resolution_optimize(torque_speed_function, inertia, target_theta):
    state = MotorModel.make_state(0,0,0)
    params = MotorModel.make_params(0, target_theta, torque_speed_function, inertia, 0, 13.599999999999987, 0.04, 0.0, 0.000025)
    
    import matplotlib.pyplot as plt
    import time
    
    time_step = 0.001
    simulation_length = 1
    
    # Graular search for best params
    total_sim_time = simulation_length / time_step
    P_resolution = 20
    I_resolution = 0.2
    D_resolution = 0.1
    P_range = np.arange(0, 200, P_resolution)
    I_range = np.arange(0, 2, I_resolution)
    D_range = np.arange(0, 0.5, D_resolution)
    
    time_metric = total_sim_time * len(P_range) * len(I_range) * len(D_range)
    rough_time_est = time_metric * 1.323004492034009e-05
    print(f"Rough time estimate: {rough_time_est}s")
    start_time = time.time()
    best_params, best_state_info_array = optimize_pid_params(P_range, D_range, I_range, params, state, time_step, simulation_length)
    total_time_taken = time.time() - start_time
    time_per_step = total_time_taken / rough_time_est
    print(f"Time per step {time_per_step}")
    
    # Fine tune params
    fine_step_divisor = 5
    P_range = np.arange(best_params['P'] - P_resolution, best_params['P'] + P_resolution, P_resolution / fine_step_divisor)
    I_range = np.arange(best_params['I'] - I_resolution, best_params['I'] + I_resolution, I_resolution / fine_step_divisor)
    D_range = np.arange(best_params['D'] - D_resolution, best_params['D'] + D_resolution, D_resolution / fine_step_divisor)
    time_metric = total_sim_time * len(P_range) * len(I_range) * len(D_range)
    rough_time_est = time_metric * 1.323004492034009e-05
    print(f"Rough time estimate: {rough_time_est}s")
    start_time = time.time()
    best_params, best_state_info_array = optimize_pid_params(P_range, D_range, I_range, best_params, state, time_step, simulation_length)
    total_time_taken = time.time() - start_time
    time_per_step = total_time_taken / rough_time_est
    print(f"Time per step {time_per_step}")
    
    # Create vertical subplots for theta, omega, and tau_out
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 8))
    
    # Plot theta
    ax1.plot(best_state_info_array['time'], best_state_info_array['theta'])
    ax1.set_ylabel('Theta (rad)')
    ax1.set_title('Theta vs Time')
    ax1.grid(True)
    
    # Plot omega
    ax2.plot(best_state_info_array['time'], best_state_info_array['omega'])
    ax2.set_ylabel('Omega (rad/s)')
    ax2.set_title('Omega vs Time')
    ax2.grid(True)
    
    # Plot tau_out
    ax3.plot(best_state_info_array['time'], best_state_info_array['tau_out'])
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Tau_out (N⋅m)')
    ax3.set_title('Tau_out vs Time')
    ax3.grid(True)
    
    # Plot error_v
    ax4.plot(best_state_info_array['time'], best_state_info_array['error_v'])
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error_v (rad/s)')
    ax4.set_title('Error_v vs Time')
    ax4.grid(True)
    
    # Plot error_p
    ax5.plot(best_state_info_array['time'], best_state_info_array['error_p'])
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Error_p (rad)')
    ax5.set_title('Error_p vs Time')
    ax5.grid(True)
    
    plt.tight_layout()
    plt.show()

def optimize_and_plot():
    state = MotorModel.make_state(0,0,0)
    max_rpm = 1000 * 2 * np.pi / 60
    max_torque = 0.1
    torque_speed_function = MotorModel.make_motor_torque_speed(max_rpm, max_torque)
    params = MotorModel.make_params(0, 2, MotorModel.make_motor_torque_speed(max_rpm, max_torque), 0.0001, 0, 13.599999999999987, 0.04, 0.0)
    
    import matplotlib.pyplot as plt
    import time
    
    time_step = 0.01
    simulation_length = 5
    
    total_sim_time = simulation_length / time_step
    P_range = np.arange(10, 14, 0.1)
    I_range = np.arange(0, 1, 0.1)
    D_range = np.arange(0.04, 0.07, .01)
    time_metric = total_sim_time * len(P_range) * len(I_range) * len(D_range)
    rough_time_est = time_metric * 1.323004492034009e-05
    print(f"Rough time estimate: {rough_time_est}s")
    start_time = time.time()
    best_params, best_state_info_array = optimize_pid_params(P_range, D_range, I_range, params, state, time_step, simulation_length)
    total_time_taken = time.time() - start_time
    time_per_step = total_time_taken / rough_time_est
    print(f"Time per step {time_per_step}")
    
    # Create vertical subplots for theta, omega, and tau_out
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 8))
    
    # Plot theta
    ax1.plot(best_state_info_array['time'], best_state_info_array['theta'])
    ax1.set_ylabel('Theta (rad)')
    ax1.set_title('Theta vs Time')
    ax1.grid(True)
    
    # Plot omega
    ax2.plot(best_state_info_array['time'], best_state_info_array['omega'])
    ax2.set_ylabel('Omega (rad/s)')
    ax2.set_title('Omega vs Time')
    ax2.grid(True)
    
    # Plot tau_out
    ax3.plot(best_state_info_array['time'], best_state_info_array['tau_out'])
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Tau_out (N⋅m)')
    ax3.set_title('Tau_out vs Time')
    ax3.grid(True)
    
    # Plot error_v
    ax4.plot(best_state_info_array['time'], best_state_info_array['error_v'])
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error_v (rad/s)')
    ax4.set_title('Error_v vs Time')
    ax4.grid(True)
    
    # Plot error_p
    ax5.plot(best_state_info_array['time'], best_state_info_array['error_p'])
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Error_p (rad)')
    ax5.set_title('Error_p vs Time')
    ax5.grid(True)
    
    plt.tight_layout()
    plt.show()

def test_motor_model_and_plot():
    state = MotorModel.make_state(0,0,0)
    max_rpm = 1000 * 2 * np.pi / 60
    max_torque = 0.10
    torque_speed_function = MotorModel.make_motor_torque_speed(max_rpm, max_torque)
    params = MotorModel.make_params(0, 2, MotorModel.make_motor_torque_speed(max_rpm, max_torque), 0.0001, 0, 13.599999999999987, 0.04, 0.0)
    
    import matplotlib.pyplot as plt
    import time
    
    time_step = 0.01
    simulation_length = 5
    
    best_state_info_array = test_motor_model(params, state, time_step, simulation_length)
    print(f"Time to settle: {evaluate_settling_time(best_state_info_array, params['target_theta'])}")
    
    # Create vertical subplots for theta, omega, and tau_out
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 8))
    
    # Plot theta
    ax1.plot(best_state_info_array['time'], best_state_info_array['theta'])
    ax1.set_ylabel('Theta (rad)')
    ax1.set_title('Theta vs Time')
    ax1.grid(True)
    
    # Plot omega
    ax2.plot(best_state_info_array['time'], best_state_info_array['omega'])
    ax2.set_ylabel('Omega (rad/s)')
    ax2.set_title('Omega vs Time')
    ax2.grid(True)
    
    # Plot tau_out
    ax3.plot(best_state_info_array['time'], best_state_info_array['tau_out'])
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Tau_out (N⋅m)')
    ax3.set_title('Tau_out vs Time')
    ax3.grid(True)
    
    # Plot error_v
    ax4.plot(best_state_info_array['time'], best_state_info_array['error_v'])
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error_v (rad/s)')
    ax4.set_title('Error_v vs Time')
    ax4.grid(True)
    
    # Plot error_p
    ax5.plot(best_state_info_array['time'], best_state_info_array['error_p'])
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Error_p (rad)')
    ax5.set_title('Error_p vs Time')
    ax5.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    dual_resolution_optimize(MotorModel.make_motor_torque_speed(1000 * 2 * np.pi / 60, 1), 0.00001, 2)