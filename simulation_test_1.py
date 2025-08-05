import numpy as np
import pandas as pd
from force_arrow import ForceArrowWindow
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import matplotlib.pyplot as plt

def calc_angle_between_vectors(v, w):
    angle = np.arctan2(v[0] * w[1] - v[1] * w[0], v[0] * w[0] + v[1] * w[1])
    return angle

def calc_rotate_vector(v, angle):
    return np.array([
        np.cos(angle) * v[0] - np.sin(angle) * v[1],
        np.sin(angle) * v[0] + np.cos(angle) * v[1], 
        0.0
    ])

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


def step(state, dt, params):
    """
    Single simulation step that takes current state and returns next state.
    
    Args:
        state: dict containing current state variables
            - A: pivot point position
            - B: blade tip position  
            - omega: angular velocity vector
        dt: time step
        params: dict containing simulation parameters
            - V_inf: freestream velocity
            - rho_inf: air density
            - c_len: chord length
            - area: blade area
            - inertia: moment of inertia
            - angle_command: angle of attack command
    
    Returns:
        dict: updated state for next step
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
    
    # Store state information
    state_information = {
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
    }
    
    
    # Calculate relative position vector
    r = B - A
    
    # Calculate velocity at blade tip
    V = - np.cross(omega, r)
    V_rel = V + V_inf

    if np.linalg.norm(V_rel) != 0.0:
        
        # Calculate dynamic pressure
        q_inf = 0.5 * rho_inf * np.linalg.norm(V_rel) ** 2
        
        # Calculate optimal angle command
        perpendicular_to_r = calc_rotate_vector(r, 90 * np.pi / 180)
        perpendicular_to_r = perpendicular_to_r / np.linalg.norm(perpendicular_to_r) * c_len
        
        optimal_vector = - V_rel / np.linalg.norm(V_rel) * c_len
        optimal_angle_command = calc_angle_between_vectors(perpendicular_to_r, optimal_vector)
        
        # print(f"Optimal angle command: {optimal_angle_command * 180 / np.pi}")
        
        # Calculate chord vector
        c = perpendicular_to_r
        angle_from_r = angle_command
        c = calc_rotate_vector(c, angle_from_r)
        
        # Calculate angle of attack
        aoa = calc_angle_between_vectors(V_rel, -c)
        if aoa > np.pi / 2:
            aoa = np.pi - aoa
        
        # Get aerodynamic coefficients
        C_l = get_lift_coefficient(aoa)
        C_d = get_drag_coefficient(aoa)
        
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
        new_omega = omega.copy()
        new_omega[2] = omega_rotation + alpha_rotation * dt
    
    # Update angular position (simplified rotation)
    theta = new_omega * dt
    # 
    r_new = calc_rotate_vector(r, theta[2])
    B_new = A + r_new
    new_state = {
        'A': A,
        'B': B_new,
        'omega': new_omega
    }
    
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
    
    # Return updated state
    return new_state, state_information


# def simulate_data():
#     # Initial state
#     state = {
#         'A': np.array([0.0, 0.0, 0.0]),
#         'B': np.array([0.0, 1.0, 0.0]),  # turbine_radius = 1
#         'omega': np.array([0.0, 0.0, 50.0])  # start_speed = 1
#     }
    
#     # Simulation parameters
#     params = {
#         'V_inf': np.array([4.0, 0.0, 0.0]),
#         'rho_inf': 1.0,
#         'c_len': 0.1,
#         'area': 0.1 * 0.1,  # c_len * 0.1
#         'inertia': np.array([0.1, 0.1, 5]),
#         'angle_command': 45* np.pi / 180  # 1 degree in radians
#     }
    
#     dt = 0.1
    
#     print("Initial state:", state['B'])
    
#     # Simulation loop
#     for i in range(100):
#         state, state_information = step(state, dt, params)
#         print(f"Step {i+1}:", state['B'])
    
#     return state, state_information


def simulate_data_with_animation():
    """Run simulation with animated visualization using force_arrow module."""
    # Create the visualization window first to initialize Qt app
    viz = ForceArrowWindow()
    df = pd.DataFrame()
    
    # Process events to ensure Qt app is fully initialized
    viz.app.processEvents()
    
    # Create a single graph window with multiple subplots
    combined_win = pg.GraphicsLayoutWidget(title="VAWT Simulation Data")
    combined_win.resize(1200, 800)
    combined_win.show()
    
    # Create subplots in a 2x3 grid layout
    # Row 0, Column 0: Torque
    torque_plot = combined_win.addPlot(row=0, col=0, title="Torque T vs Time")
    torque_plot.setLabel('left', 'Torque T (N⋅m)')
    torque_plot.setLabel('bottom', 'Time (s)')
    torque_plot.showGrid(x=True, y=True)
    
    # Row 0, Column 1: Drag Coefficient
    cd_plot = combined_win.addPlot(row=0, col=1, title="Drag Coefficient (CD) vs Time")
    cd_plot.setLabel('left', 'CD')
    cd_plot.setLabel('bottom', 'Time (s)')
    cd_plot.showGrid(x=True, y=True)
    
    # Row 0, Column 2: Lift Coefficient
    cl_plot = combined_win.addPlot(row=0, col=2, title="Lift Coefficient (CL) vs Time")
    cl_plot.setLabel('left', 'CL')
    cl_plot.setLabel('bottom', 'Time (s)')
    cl_plot.showGrid(x=True, y=True)
    
    # Row 1, Column 0: Angle of Attack
    aoa_plot = combined_win.addPlot(row=1, col=0, title="Angle of Attack (AOA) vs Time")
    aoa_plot.setLabel('left', 'AOA (degrees)')
    aoa_plot.setLabel('bottom', 'Time (s)')
    aoa_plot.showGrid(x=True, y=True)
    
    # Row 1, Column 1: Angle Command
    angle_command_plot = combined_win.addPlot(row=1, col=1, title="Angle Command vs Time")
    angle_command_plot.setLabel('left', 'Angle Command (degrees)')
    angle_command_plot.setLabel('bottom', 'Time (s)')
    angle_command_plot.showGrid(x=True, y=True)
    
    # Row 1, Column 2: Angular Velocity
    angular_velocity_plot = combined_win.addPlot(row=1, col=2, title="Angular Velocity vs Time")
    angular_velocity_plot.setLabel('left', 'Angular Velocity (rad/s)')
    angular_velocity_plot.setLabel('bottom', 'Time (s)')
    angular_velocity_plot.showGrid(x=True, y=True)
    
    # Row 2, Column 0: Power
    power_plot = combined_win.addPlot(row=2, col=0, title="Power vs Time")
    power_plot.setLabel('left', 'Power (W)')
    power_plot.setLabel('bottom', 'Time (s)')
    power_plot.showGrid(x=True, y=True)
    power_plot.addLegend()
    
    # Row 2, Column 1: Power Running Average
    power_avg_plot = combined_win.addPlot(row=2, col=1, title="Power Running Average vs Time")
    power_avg_plot.setLabel('left', 'Power (W)')
    power_avg_plot.setLabel('bottom', 'Time (s)')
    power_avg_plot.showGrid(x=True, y=True)
    power_avg_plot.addLegend()
    
    # Initialize curves for plotting
    torque_curve = torque_plot.plot(pen='red', name='Torque T')
    cd_curve = cd_plot.plot(pen='green', name='CD')
    cl_curve = cl_plot.plot(pen='blue', name='CL')
    aoa_curve = aoa_plot.plot(pen='orange', name='AOA')
    angle_command_curve = angle_command_plot.plot(pen='purple', name='Angle Command')
    angular_velocity_curve = angular_velocity_plot.plot(pen='brown', name='Angular Velocity')
    power_curve = power_plot.plot(pen='yellow', name='Power')
    power_avg_curve = power_avg_plot.plot(pen='red', name='Power Running Average')
    # Initial state
    state = {
        'A': np.array([0.0, 0.0, 0.0]),
        'B': np.array([0.0, -1.0, 0.0]),  # turbine_radius = 1   
        'omega': np.array([0.0, 0.0, 0.0])  # start_speed = -0.3
    }
    
    # Simulation parameters
    params = {
        'V_inf': np.array([4.4, 0.0, 0.0]),
        'rho_inf': 1.2,
        'c_len': 0.1,
        'area': 0.1 * 0.1,  # c_len * 0.1
        'inertia': np.array([0.1, 0.1, 0.01]),
        'angle_command': 3 * np.pi / 180  # 1 degree in radians
    }
    
    dt = 0.01
    seconds = 40
    steps = int(seconds / dt)
    
    # Running average parameters
    window_size = 100  # Number of points for running average
    power_history = []  # Store power values for running average calculation
    power_avg_history = []  # Store running average values
    power_avg_times = []  # Store times for running average
    
    # Add bodies to visualization
    body_A = viz.add_body(state['A'][0], state['A'][1], 1, 'blue', "A (Pivot)")
    body_B = viz.add_body(state['B'][0], state['B'][1], 1, 'red', "B (Blade Tip)")
    
    # Simulation loop with animation
    for i in range(steps):
        current_time = i * dt
        if i + 1 > steps:
            break
        # Store previous state for force calculation
        prev_state = state.copy()
        
        
        # Update simulation
        corrected_angle_command = params['angle_command']
        if True:
            best_angle_command = 0
            best_value = -np.inf
            for z in np.linspace(-90, 90, 20):
                test_params = params.copy()
                test_params['angle_command'] = z * np.pi / 180
                _, state_information_optimized = step(state.copy(), dt, test_params)
                new_value = state_information_optimized['T'][2]
                if new_value > best_value:
                    best_value = new_value
                    best_angle_command = z * np.pi / 180
            corrected_angle_command = best_angle_command
            
        params['angle_command'] = corrected_angle_command
        
        state, state_information = step(state.copy(), dt, params)

        # Add time to state_information for dataframe
        state_information['angle_command'] = params['angle_command'] * 180 / np.pi
        state_information['time'] = current_time
        df = df._append(state_information, ignore_index=True)
        
        # Update plots using dataframe data (only show last 1000 points to prevent memory issues)
        if len(df) > steps:
            df = df.tail(steps)
        
        # Update all plots with dataframe data
        if len(df) > 0:
            torque_curve.setData(df['time'], df['T'].apply(lambda x: x[2]))
            cd_curve.setData(df['time'], df['C_d'])
            cl_curve.setData(df['time'], df['C_l'])
            aoa_curve.setData(df['time'], df['aoa'] * 180 / np.pi)
            angle_command_curve.setData(df['time'], df['angle_command'])
            angular_velocity_curve.setData(df['time'], df['omega'])
            power_curve.setData(df['time'], df['power'])
            
            # Calculate and plot running average of power
            power_history.append(state_information['power'])
            if len(power_history) > window_size:
                power_history.pop(0)  # Remove oldest value
            
            if len(power_history) > 0:
                power_avg = np.mean(power_history)
                power_avg_history.append(power_avg)
                power_avg_times.append(current_time)
                
                # Keep only the last 1000 points to prevent memory issues
                if len(power_avg_history) > 1000:
                    power_avg_history = power_avg_history[-1000:]
                    power_avg_times = power_avg_times[-1000:]
                
                # Plot the running average time series
                power_avg_curve.setData(power_avg_times, power_avg_history)
        # Update body positions in visualization
        viz.update_body_position(0, state['A'][0], state['A'][1])  # Point A
        viz.update_body_position(1, state['B'][0], state['B'][1])  # Point B
        
        # Clear previous forces
        viz.clear_forces()
        
        # Add velocity vector at point B
        r = state_information['r']
        V = state_information['V']
        viz.add_force(body_B, V[0], V[1], 'green', 1, "Velocity")
        
        # Add freestream velocity vector
        viz.add_force(body_B, params['V_inf'][0], params['V_inf'][1], 'orange', 1, "V∞")
        
        # Add relative velocity vector
        V_rel = state_information['V_rel']
        viz.add_force(body_B, V_rel[0], V_rel[1], 'purple', 1, "V_rel")
        
        # Add angular velocity indicator at point A
        viz.add_force(body_A, 0, state_information['omega'], 'cyan', 1, "ω")
        viz.add_force(body_A, 0, state_information['alpha'], 'red', 1, "α")
        
        force_scale = 1
        viz.add_force(body_B, state_information['L'][0], state_information['L'][1], 'red', force_scale, "L")
        viz.add_force(body_B, state_information['D'][0], state_information['D'][1], 'blue', force_scale, "D")
        viz.add_force(body_B, state_information['E'][0], state_information['E'][1], 'green', force_scale, "E")
        viz.add_force(body_B, state_information['R'][0], state_information['R'][1], 'yellow', force_scale, "R")
        viz.add_force(body_B, state_information['F'][0], state_information['F'][1], 'purple', force_scale, "F")
        
        # Add chord vector
        viz.add_force(body_B, state_information['c'][0], state_information['c'][1], 'white', 10, "c")
        
        # Show total forces
        viz.show_total_forces(0.5)
        
        print(f"Step {i+1}:", state['B'], "at time ", current_time)
        
        # Small delay for animation
        #time.sleep(0.05)
        # Process Qt events to keep the window responsive
        viz.app.processEvents()
    
    return state, state_information, viz, df, combined_win


def plot_cd_cl_vs_aoa():
    angles = np.linspace(180 * np.pi / 180, -180 * np.pi / 180, 200)
    cds = []
    cls = []
    for angle in angles:
        cds.append(get_drag_coefficient(angle))
        cls.append(get_lift_coefficient(angle))
    plt.plot(angles, cds, label='CD')
    plt.plot(angles, cls, label='CL')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # print("checking cd and cl vs aoa")
    # plot_cd_cl_vs_aoa()
    
    # Run the animated simulation
    final_state, final_state_info, viz, df, combined_win = simulate_data_with_animation()
    print(f"\nFinal angular velocity: {final_state['omega']}")
    
    # Keep the windows open
    print("Animation complete! Close the windows to exit.")
    viz.app.exec_()