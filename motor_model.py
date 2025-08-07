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
    
    def make_params(time, target_theta, torque_speed, inertia, input_torque, P, D, I):
        new_params = {
            'time': time,
            'target_theta': target_theta,
            'torque_speed': torque_speed,
            'inertia': inertia,
            'input_torque': input_torque,
            'P': P,
            'D': D,
            'I': I
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
    
    def step(state, params, dt):
        error_p = params['target_theta'] - state['theta']
        target_omega = params['P'] * error_p
        error_v = target_omega - state['omega']
        error_i = state['error_i'] + error_v * dt
        tau_out = params['I'] * error_i + params['D'] * error_v
        tau_out = params['torque_speed'](state['omega'], tau_out)
        alpha = tau_out / params['inertia']
        omega = state['omega'] + alpha * dt
        theta = state['theta'] + omega * dt
        new_state_info = MotorModel.make_state_info(params['time'], theta, omega, alpha, error_p, error_v, error_i, tau_out, target_omega)
        new_state = MotorModel.make_state(theta, omega, error_i)
        
        return new_state, new_state_info
    
    
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

def optimize_pid_params(time_range, P_range, D_range, I_range, params, state, time_step):
    best_state_info_array = {}
    best_params = params
    best_final_error = np.inf
    best_final_vel = np.inf
    for time in time_range:
        for P in P_range:
            for D in D_range:
                for I in I_range:
                    test_params = copy.deepcopy(params)
                    test_params['P'] = P
                    test_params['D'] = D
                    test_params['I'] = I
                    state_info_array = test_motor_model(test_params, state, time_step, time)
                    final_error = abs(state_info_array['error_p'][-1])
                    final_vel = abs(state_info_array['error_v'][-1])
                    # print(f"Test params: P={P}, D={D}, I={I}, time={time}, final_error={final_error}", end = " - ")
                    if final_error < best_final_error and best_final_error > 0.05 * params['target_theta'] and final_vel < best_final_vel and best_final_vel > 0.05:
                        best_final_error = final_error
                        best_params = test_params
                        best_state_info_array = state_info_array
                        best_final_vel = final_vel
                        # print(f"New best params: P={P}, D={D}, I={I}, time={time}, final_error={final_error}")
                        
    print(f"Best params: P={best_params['P']}, D={best_params['D']}, I={best_params['I']}, final_error={best_final_error}")
    return best_params, best_state_info_array

if __name__ == "__main__":
    state = MotorModel.make_state(0,0,0)
    max_rpm = 1000 * 2 * np.pi / 60
    max_torque = 0.1
    torque_speed_function = MotorModel.make_motor_torque_speed(max_rpm, max_torque)
    params = MotorModel.make_params(0, 0.1, torque_speed_function, 0.001, 0, 191, 1, 26)
    
    import matplotlib.pyplot as plt
    import time
    
    time_step = 0.001
    simulation_length = 5
    num_steps = int(simulation_length / time_step)
    cur_time = 0
    
    time_range = np.arange(0.1, 1, 0.1)
    total_sim_time = sum(time_range) / time_step
    P_range = np.arange(0.1, 30, 5)
    I_range = np.arange(0, 10, 1)
    D_range = np.arange(0.01, 0.3, .05)
    rough_time_est = total_sim_time * len(P_range) * len(I_range) * len(D_range) * 1.1259823375278045e-05
    print(f"Rough time estimate: {rough_time_est}s")
    start_time = time.time()
    best_params, best_state_info_array = optimize_pid_params(time_range, P_range, D_range, I_range, params, state, time_step)
    #best_state_info_array = test_motor_model(params, state, time_step, simulation_length)
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