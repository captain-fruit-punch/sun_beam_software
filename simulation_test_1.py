import numpy as np
import pandas as pd
from force_arrow import ForceArrowWindow
import time
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import matplotlib.pyplot as plt
from motor_model import MotorModel
from wind_turbine_blade import WindTurbineBlade

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
    
    # Row 2, Column 2: Motor Torque
    motor_torque_plot = combined_win.addPlot(row=2, col=2, title="Motor Torque vs Time")
    motor_torque_plot.setLabel('left', 'Motor Torque (N⋅m)')
    motor_torque_plot.setLabel('bottom', 'Time (s)')
    motor_torque_plot.showGrid(x=True, y=True)
    motor_torque_plot.addLegend()
    
    # Initialize curves for plotting
    torque_curve = torque_plot.plot(pen='red', name='Torque T')
    cd_curve = cd_plot.plot(pen='green', name='CD')
    cl_curve = cl_plot.plot(pen='blue', name='CL')
    aoa_curve = aoa_plot.plot(pen='orange', name='AOA')
    angle_command_curve = angle_command_plot.plot(pen='purple', name='Angle Command')
    motor_theta_curve = angle_command_plot.plot(pen='brown', name='Motor Theta')
    angular_velocity_curve = angular_velocity_plot.plot(pen='brown', name='Angular Velocity')
    power_curve = power_plot.plot(pen='yellow', name='Power')
    power_avg_curve = power_avg_plot.plot(pen='red', name='Power Running Average')
    motor_torque_curve = motor_torque_plot.plot(pen='blue', name='Motor Torque')
    
    # Initial state
    motor_state = MotorModel.make_state(0,0,0)
    motor_params = MotorModel.make_params(0, 2, MotorModel.make_motor_torque_speed(1000 * 2 * np.pi / 60, 1), 0.001, 0, 20.0, 0.05, 0.84)
    
    state = WindTurbineBlade.make_state(
        A=np.array([0.0, 0.0, 0.0]),
        B=np.array([0.0, -1.0, 0.0]),  # turbine_radius = 1   
        omega=np.array([0.0, 0.0, 0.0]),  # start_speed = -0.3
        motor_state=motor_state
    )
    
    # Simulation parameters
    params = WindTurbineBlade.make_params(
        time=0,
        V_inf=np.array([4.4, 0.0, 0.0]),
        rho_inf=1.2,
        c_len=0.1,
        area=0.1 * 0.1,  # c_len * 0.1
        inertia=np.array([0.1, 0.1, 0.01]),
        angle_command=3 * np.pi / 180,  # 1 degree in radians
        motor_params=motor_params
    )
    
    state_info_array = {}
    motor_state_info_array = {}
    
    dt = 0.02
    seconds = 40
    steps = int(seconds / dt)
    
    # Running average parameters
    window_duration = 10
    window_size = int(window_duration / dt)
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
        params['motor_params']['time'] = current_time
        params['time'] = current_time
        # Update simulation
        corrected_angle_command = params['angle_command']
        if True:
            best_angle_command = 0
            best_value = -np.inf
            for z in np.linspace(-90, 90, 20):
                test_params = params.copy()
                test_params['angle_command'] = z * np.pi / 180
                _, state_information_optimized = WindTurbineBlade.step(state.copy(), dt, test_params, simulate_motor=False)
                new_value = state_information_optimized['T']
                if new_value > best_value:
                    best_value = new_value
                    best_angle_command = z * np.pi / 180
            corrected_angle_command = best_angle_command
            
        params['angle_command'] = corrected_angle_command
        # Use RK4 for higher accuracy
        params['integrator'] = 'rk4'
        params['motor_params']['integrator'] = 'rk4'
        
        state, state_information = WindTurbineBlade.step(state.copy(), dt, params)
        # Add derived fields to state_information before copying to arrays
        state_information['angle_command'] = params['angle_command'] * 180 / np.pi
        state_info_array = WindTurbineBlade.copy_state_info_to_array(state_information, state_info_array)
        #print(state_information['motor_state_info'])
        motor_state_info_array = MotorModel.copy_state_info_to_array(state_information['motor_state_info'], motor_state_info_array)
        
        # Update all plots with dataframe data
        if len(state_info_array) > 0:
            #print(motor_state_info_array)
            torque_curve.setData(state_info_array['time'], state_info_array['T'])
            cd_curve.setData(state_info_array['time'], state_info_array['C_d'])
            cl_curve.setData(state_info_array['time'], state_info_array['C_l'])
            aoa_curve.setData(state_info_array['time'], np.array(state_info_array['aoa']) * 180 / np.pi)
            angle_command_curve.setData(state_info_array['time'], state_info_array['angle_command'])
            motor_theta_curve.setData(motor_state_info_array['time'], np.array(motor_state_info_array['theta']) * (180 / np.pi))
            angular_velocity_curve.setData(state_info_array['time'], state_info_array['omega'])
            power_curve.setData(state_info_array['time'], state_info_array['power'])
            motor_torque_curve.setData(motor_state_info_array['time'], np.array(motor_state_info_array['tau_out']))
            
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
        viz.add_force(body_A, 0, state_information['alpha'][2], 'red', 1, "α")
        
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
    
    return state, state_information, viz, state_info_array, combined_win


def plot_cd_cl_vs_aoa():
    angles = np.linspace(180 * np.pi / 180, -180 * np.pi / 180, 200)
    cds = []
    cls = []
    for angle in angles:
        cds.append(WindTurbineBlade.get_drag_coefficient(angle))
        cls.append(WindTurbineBlade.get_lift_coefficient(angle))
    plt.plot(angles, cds, label='CD')
    plt.plot(angles, cls, label='CL')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # print("checking cd and cl vs aoa")
    # plot_cd_cl_vs_aoa()
    
    # Run the animated simulation
    final_state, final_state_info, viz, state_info_array, combined_win = simulate_data_with_animation()
    print(f"\nFinal angular velocity: {final_state['omega']}")
    
    # Keep the windows open
    print("Animation complete! Close the windows to exit.")
    viz.app.exec_()