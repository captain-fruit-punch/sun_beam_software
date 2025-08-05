import numpy as np
import pandas as pd
from force_arrow import ForceArrowWindow
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

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
    return float(np.interp(alpha_abs, alphas, cls))


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
    return float(np.interp(alpha_abs, alphas, cds))


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
    }
    
    
    # Calculate relative position vector
    r = B - A
    
    # Calculate velocity at blade tip
    V = - np.cross(omega, r)
    V_rel = V + V_inf
    
    if np.linalg.norm(V_rel) != 0:
        # Calculate dynamic pressure
        q_inf = 0.5 * rho_inf * np.linalg.norm(V_rel) ** 2
        
        # Calculate optimal angle command
        c = calc_rotate_vector(r, -90 * np.pi / 180)
        c = c / np.linalg.norm(c) * c_len
        optimal_angle_command = calc_angle_between_vectors(V_rel, c)
        
        print(f"Optimal angle command: {optimal_angle_command * 180 / np.pi}")
        
        # Calculate chord vector
        angle_from_r = optimal_angle_command
        c = calc_rotate_vector(c, angle_from_r)
        
        # Calculate angle of attack
        aoa = calc_angle_between_vectors(V_rel, -c) + angle_command * np.pi / 180
        if aoa > np.pi / 2:
            aoa = np.pi - aoa
        
        # Get aerodynamic coefficients
        C_l = get_lift_coefficient(aoa)
        C_d = get_drag_coefficient(aoa)
        
        # Calculate forces
        sign_of_aoa = np.sign(aoa)
        lift_direction = np.array([
            np.cos(-90 * np.pi / 180) * sign_of_aoa * V_rel[0] - np.sin(-90 * np.pi / 180) * sign_of_aoa * V_rel[1],
            np.sin(-90 * np.pi / 180) * sign_of_aoa * V_rel[0] + np.cos(-90 * np.pi / 180) * sign_of_aoa * V_rel[1], 
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
        omega_new = omega + alpha * dt
        
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
    
    # Update angular position (simplified rotation)
    theta = omega * dt
    # 
    r_new = np.array([
            np.cos(theta[2]) * r[0] - np.sin(theta[2]) * r[1],
            np.sin(theta[2]) * r[0] + np.cos(theta[2]) * r[1], 
            0.0
        ])
    B_new = A + r_new
    
    new_state = {
        'A': A,
        'B': B_new,
        'omega': omega_new
    }
    
    
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
    # Create the visualization window
    viz = ForceArrowWindow()
    df = pd.DataFrame()
    
    # Create separate graph windows for different variables
    # Torque graph
    torque_win = pg.GraphicsLayoutWidget(title="Torque T vs Time")
    torque_win.resize(800, 400)
    torque_win.show()
    
    # CD graph
    cd_win = pg.GraphicsLayoutWidget(title="Drag Coefficient (CD) vs Time")
    cd_win.resize(800, 400)
    cd_win.show()
    
    # CL graph
    cl_win = pg.GraphicsLayoutWidget(title="Lift Coefficient (CL) vs Time")
    cl_win.resize(800, 400)
    cl_win.show()
    
    # AOA graph
    aoa_win = pg.GraphicsLayoutWidget(title="Angle of Attack (AOA) vs Time")
    aoa_win.resize(800, 400)
    aoa_win.show()
    
    # Create the plots
    torque_plot = torque_win.addPlot(title="Torque T Over Time")
    torque_plot.setLabel('left', 'Torque T (N⋅m)')
    torque_plot.setLabel('bottom', 'Time (s)')
    torque_plot.showGrid(x=True, y=True)
    
    cd_plot = cd_win.addPlot(title="Drag Coefficient Over Time")
    cd_plot.setLabel('left', 'CD')
    cd_plot.setLabel('bottom', 'Time (s)')
    cd_plot.showGrid(x=True, y=True)
    
    cl_plot = cl_win.addPlot(title="Lift Coefficient Over Time")
    cl_plot.setLabel('left', 'CL')
    cl_plot.setLabel('bottom', 'Time (s)')
    cl_plot.showGrid(x=True, y=True)
    
    aoa_plot = aoa_win.addPlot(title="Angle of Attack Over Time")
    aoa_plot.setLabel('left', 'AOA (degrees)')
    aoa_plot.setLabel('bottom', 'Time (s)')
    aoa_plot.showGrid(x=True, y=True)
    
    # Initialize curves for plotting
    torque_curve = torque_plot.plot(pen='red', name='Torque T')
    cd_curve = cd_plot.plot(pen='green', name='CD')
    cl_curve = cl_plot.plot(pen='blue', name='CL')
    aoa_curve = aoa_plot.plot(pen='orange', name='AOA')
    
    # Initial state
    state = {
        'A': np.array([0.0, 0.0, 0.0]),
        'B': np.array([0.0, 1.0, 0.0]),  # turbine_radius = 1   
        'omega': np.array([0.0, 0.0, 40.0])  # start_speed = -0.3
    }
    
    # Simulation parameters
    params = {
        'V_inf': np.array([4.4, 0.0, 0.0]),
        'rho_inf': 1.0,
        'c_len': 0.1,
        'area': 0.1 * 0.1,  # c_len * 0.1
        'inertia': np.array([0.1, 0.1, 0.1]),
        'angle_command': 1 * np.pi / 180  # 1 degree in radians
    }
    
    dt = 0.001
    seconds = 1000
    steps = int(seconds / dt)
    
    # Add bodies to visualization
    body_A = viz.add_body(state['A'][0], state['A'][1], 1, 'blue', "A (Pivot)")
    body_B = viz.add_body(state['B'][0], state['B'][1], 1, 'red', "B (Blade Tip)")
    
    print("Initial state:", state['B'])
    
    # Simulation loop with animation
    for i in range(steps):
        current_time = i * dt
        if i + 1 > steps:
            break
        # Store previous state for force calculation
        prev_state = state.copy()
        
        # Update simulation
        state, state_information = step(state, dt, params)
        
        # Add time to state_information for dataframe
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
        viz.add_force(body_A, 0, state['omega'][2], 'cyan', 1, "ω")
        
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
        
        print(f"Step {i+1}:", state['B'])
        
        # Small delay for animation
        #time.sleep(0.001)
        # Process Qt events to keep the window responsive
        viz.app.processEvents()
    
    return state, state_information, viz, df, [torque_win, cd_win, cl_win, aoa_win]


if __name__ == "__main__":
    # Run the animated simulation
    final_state, final_state_info, viz, df, graph_windows = simulate_data_with_animation()
    print(f"\nFinal angular velocity: {final_state['omega']}")
    
    # Keep the windows open
    print("Animation complete! Close the windows to exit.")
    viz.app.exec_()