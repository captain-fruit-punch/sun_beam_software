import numpy as np
from wind_turbine_blade import WindTurbineBlade
from force_arrow import ForceArrowWindow

def calc_single_chord_lift(chord_v, vel_vec, rho, height):
    aoa = WindTurbineBlade.calc_angle_between_vectors(vel_vec, -chord_v)
    q_inf = 0.5 * rho * np.linalg.norm(vel_vec) ** 2
    if aoa > np.pi / 2:
        aoa = np.pi - aoa
    print(f"aoa: {aoa}")
    area = np.linalg.norm(chord_v) * height
    # Coefficients
    C_l = WindTurbineBlade.get_lift_coefficient(aoa)
    C_d = WindTurbineBlade.get_drag_coefficient(aoa)
    lift_direction = np.array([
        np.cos(-90 * np.pi / 180) * vel_vec[0] - np.sin(-90 * np.pi / 180) * vel_vec[1],
        np.sin(-90 * np.pi / 180) * vel_vec[0] + np.cos(-90 * np.pi / 180) * vel_vec[1], 
        0.0
    ])
    L = C_l * q_inf * area * (lift_direction / np.linalg.norm(lift_direction))
    D = C_d * q_inf * area * (vel_vec / np.linalg.norm(vel_vec))
    return L, D
    

if __name__ == "__main__":
    c = 1
    x_cp = c / 4
    omega_vec = np.array([0.0, 0.0, 4])
    rho = 1.225
    height = 1
    turbine_radius = 10
    
    A_vec = np.array([0, 0, 0])
    B_vec = np.array([0, turbine_radius, 0])
    r_vec = B_vec - A_vec
    c_norm_vec = WindTurbineBlade.calc_rotate_vector(-r_vec, np.pi/2)/np.linalg.norm(r_vec)
    
    num_points = 10
    x_scan = np.linspace(-x_cp, c - x_cp, num_points)
    dx = c / (num_points - 1)
    
    L_total_vec = np.array([0.0, 0.0, 0.0])
    D_total_vec = np.array([0.0, 0.0, 0.0])
    
    aoa_array = []
    
    for x in x_scan:
        r_vec = r_vec + c_norm_vec * x
        print(f"r_vec: {r_vec}")
        vel_vec = np.cross(omega_vec, r_vec)
        print(f"vel_vec: {vel_vec}")
        L, D = calc_single_chord_lift(c_norm_vec * dx, vel_vec, rho, height)
        print(f"L: {L}")
        print(f"D: {D}")
        L_total_vec += L
        D_total_vec += D
        aoa_array.append(WindTurbineBlade.calc_angle_between_vectors(vel_vec, -c_norm_vec * dx))
    
    T_total_vec = L_total_vec + D_total_vec
    
    L_comp_vec, D_comp_vec = calc_single_chord_lift(c * c_norm_vec, np.cross(omega_vec, r_vec), rho, height)
    T_comp_vec = L_comp_vec + D_comp_vec
    
    print(T_total_vec)
    print(T_comp_vec)
    print(aoa_array)
    
    viz = ForceArrowWindow()
    
    rot_axis_body = viz.add_body(A_vec[0], A_vec[1], 0.1, 'black', "Rotation Axis")
    cp_body = viz.add_body(B_vec[0], B_vec[1], 0.1, 'black', "Center of Pressure")
    viz.add_force(rot_axis_body, 0, omega_vec[2], 'yellow', 1.0, "ω")
    viz.add_force(cp_body, c_norm_vec[0] * x_cp, 0, 'white', 1.0, "c_norm")
    viz.add_force(cp_body, c_norm_vec[0] * -(c - x_cp), 0, 'white', 1.0, "c_norm")
    viz.add_force(cp_body, T_total_vec[0], T_total_vec[1], 'purple', 1.0, "T_total")
    viz.add_force(cp_body, T_comp_vec[0], T_comp_vec[1], 'red', 1.0, "T_comp")
    
    viz.show_total_forces(1.0)
    viz.app.exec_()