import numpy as np

class MotorModel:
    def make_state(theta, omega, error_i):
        new_state = {
            'theta': theta,
            'omega': omega,
            'error_i': error_i
        }
        return new_state
    
    def make_perams(time, target_theta, torque_speed, inertia, input_torque, P, D, I):
        new_perams = {
            'time': time,
            'target_theta': target_theta,
            'torque_speed': torque_speed,
            'inertia': inertia,
            'input_torque': input_torque,
            'P': P,
            'D': D,
            'I': I
        }
        return new_perams
        
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
    
    def step(state, perams, dt):
        error_p = perams['target_theta'] - state['theta']
        target_omega = perams['P'] * error_p
        error_v = target_omega - state['omega']
        error_i = state['error_i'] + error_v * dt
        tau_out = perams['I'] * error_i + perams['D'] * error_v
        alpha = tau_out / perams['inertia']
        omega = state['omega'] + alpha * dt
        theta = state['theta'] + omega * dt
        new_state_info = MotorModel.make_state_info(perams['time'], theta, omega, alpha, error_p, error_v, error_i, tau_out, target_omega)
        new_state = MotorModel.make_state(theta, omega, error_i)
        
        return new_state, new_state_info
    
if __name__ == "__main__":
    state = MotorModel.make_state(0,0,0)
    perams = MotorModel.make_perams(0, 2*np.pi, [], 0.1, 0, 1, 0.1, 0)
    
    import matplotlib.pyplot as plt
    
    time_step = 0.1
    simulation_length = 100
    num_steps = int(simulation_length / time_step)
    cur_time = 0
    
    state_info_array = {}
    
    for i in range(num_steps):
        new_state, new_state_info = MotorModel.step(state, perams, time_step)
        new_state_info_array = MotorModel.copy_state_info_to_array(new_state_info, state_info_array)
        state = new_state
        state_info_array = new_state_info_array
        perams['time'] = cur_time
        cur_time += time_step
        
    
    # Create vertical subplots for theta, omega, and tau_out
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    
    # Plot theta
    ax1.plot(state_info_array['time'], state_info_array['theta'])
    ax1.set_ylabel('Theta (rad)')
    ax1.set_title('Theta vs Time')
    ax1.grid(True)
    
    # Plot omega
    ax2.plot(state_info_array['time'], state_info_array['omega'])
    ax2.set_ylabel('Omega (rad/s)')
    ax2.set_title('Omega vs Time')
    ax2.grid(True)
    
    # Plot tau_out
    ax3.plot(state_info_array['time'], state_info_array['tau_out'])
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Tau_out (N⋅m)')
    ax3.set_title('Tau_out vs Time')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()