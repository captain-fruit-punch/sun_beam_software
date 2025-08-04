import numpy as np


def simulate_data():
    turbine_radius = 1
    start_speed = 1
    V_inf = np.array([1.0, 0.0, 0.0])
    A = np.array([0.0, 0.0, 0.0])
    B = np.array([0.0, 1.0, 0.0]) * turbine_radius
    omega = np.array([0.0, 0.0, 1.0]) * start_speed
    rho_inf = 1.0
    c_len = 0.1
    c_p = c / 4
    area = c * 0.1


    def step(dt):
        V = np.cross(omega, r)
        V_rel = V + V_inf
        q_inf = 0.5 * rho_inf * np.linalg.norm(V_rel) ** 2
        # chord vector
        angle_command = 1 * np.pi / 180  # angle of attack in radians
        ninety_deg = 90 * np.pi / 180
        angle_from_r = angle_command + ninety_deg
        r = B - A
        c = np.array([np.cos(angle_from_r) * r[0] - np.sin(angle_from_r) * r[1], np.sin(angle_from_r) * r[0] + np.cos(angle_from_r) * r[1], 0.0])
        c = c / np.linalg.norm(c) * c_len
        inertia = np.array([0.1, 0.1, 0.1])

        C_l = 2 * np.pi # implment lift coefficient
        C_d =  # implement drag coefficient

        aoa = np.arccos(np.dot(V_rel, c) / (np.linalg.norm(V_rel) * np.linalg.norm(c)))
        if aoa > np.pi / 2:
            aoa = np.pi - aoa
        # lift force
        L = C_l * q_inf * area * (c / np.linalg.norm(c))
        # drag force
        D = C_d * q_inf * area * (V_rel / np.linalg.norm(V_rel))
        # total force
        E = L + D
        # Reaction force
        R = np.dot(E, r) / (np.linalg.norm(r) ** 2) * r
        # Resultant force
        F = E - R
        # torque
        T = np.cross(r, F)
        # angular acceleration
        alpha = T / inertia
        # angular velocity update
        omega += alpha * dt
        # angular position update
        theta = omega * dt
        r_rot = r * np.cos(theta) + np.cross(omega, r) * np.sin(theta) + omega * np.dot(omega, r) * (1 - np.cos(theta))
        B = A + r_rot
        
    step(0.1)
    