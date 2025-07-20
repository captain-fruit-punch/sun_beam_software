import numpy as np
import matplotlib.pyplot as plt

def calculate_cl_and_cd(alpha_deg):
    alpha_rad = np.radians(alpha_deg)
    CL = 2 * np.pi * np.sin(alpha_rad)
    CD = 1.28 * np.sin(alpha_rad)**2
    return CL, CD

alpha_deg = np.linspace(-90, 90, 360)
CL, CD = calculate_cl_and_cd(alpha_deg)

plt.plot(alpha_deg, CL, label='CL (Lift)')
plt.plot(alpha_deg, CD, label='CD (Drag)')
plt.axhline(0, color='gray', linestyle='--')
plt.xlabel('Angle of Attack (deg)')
plt.ylabel('Coefficient')
plt.legend()
plt.grid(True)
plt.title('Flat Plate CL and CD vs. AoA')
plt.xlim(-90, 90)
plt.show()
