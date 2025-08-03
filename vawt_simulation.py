import numpy as np
import matplotlib.pyplot as plt
import vector_tools
import aoa_calculation

blade_height = 0.1
blade_radius = 0.1
rotor_speed =  2 * 2 * np.pi  # rad/s

wind_vector = np.array([4.4, 0])
air_density = 1.225

def calculate_lift_and_drag(chord_vector, wind_vector):
    aoa = vector_tools.calculate_angle_between_vectors(chord_vector, wind_vector)

    drag_unit_vector = vector_tools.calculate_unit_vector(wind_vector)
    lift_unit_vector = vector_tools.calculate_unit_vector(vector_tools.rotate_vector(wind_vector, 90))

    cl_at_aoa, cd_at_aoa = aoa_calculation.calculate_cl_and_cd(aoa)

    area_of_foil = blade_height * vector_tools.calculate_magnitude_of_vector(chord_vector)

    wind_speed = vector_tools.calculate_magnitude_of_vector(wind_vector)

    lift_magnitude = cl_at_aoa * air_density * wind_speed**2 * area_of_foil / 2
    drag_magnitude = cd_at_aoa * air_density * wind_speed**2 * area_of_foil / 2

    lift = lift_magnitude * lift_unit_vector
    drag = drag_magnitude * drag_unit_vector
    resultant_force = lift + drag

    chord_projection_of_force = vector_tools.calculate_projection(resultant_force, chord_vector) * vector_tools.calculate_unit_vector(chord_vector)
    return lift, drag, resultant_force, chord_projection_of_force

# Create angles from 0 to 10 degrees with 1 degree increments
angles = np.arange(0, 360, 2)  # 0, 1, 2, ..., 10 degrees

# Create subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

# Plot 1: Vector visualization for each angle
origin = np.array([0, 0])
colors = plt.cm.viridis(np.linspace(0, 1, len(angles)))

for i, angle in enumerate(angles):
    # Rotate chord vector by the angle (negative to match your original -5 degrees)
    chord_vector = vector_tools.rotate_vector(np.array([0.03, 0]), -angle)
    chord_velocity_component = vector_tools.calculate_unit_vector(chord_vector) * rotor_speed * blade_radius
    print("chord_velocity_component: ", chord_velocity_component)
    wind_velocity_component = wind_vector - chord_velocity_component
    
    # Calculate forces
    lift, drag, resultant_force, chord_projection_of_force = calculate_lift_and_drag(chord_vector, wind_velocity_component)
    
    # Plot chord vector
    ax1.quiver(*origin, *chord_vector, color=colors[i], angles='xy', scale_units='xy', scale=1, 
               alpha=0.7, label=f'{angle}°')
    
    # Plot chord projection of force
    ax1.quiver(*origin, *chord_projection_of_force, color=colors[i], angles='xy', scale_units='xy', scale=1,
               alpha=0.5, linestyle='--', linewidth=2)

# Plot wind vector (same for all cases)
ax1.quiver(*origin, *wind_vector, color='g', angles='xy', scale_units='xy', scale=1, 
           label='Wind Vector', linewidth=3)

ax1.set_xlim(-0.03, 0.03)
ax1.set_ylim(-0.03, 0.03)
ax1.grid(True, alpha=0.3)
ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
ax1.set_title('Chord Vector Rotation and Force Projections')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_aspect('equal', adjustable='box')

# Plot 2: Magnitude of chord projection vs angle
chord_projection_magnitudes = []
for angle in angles:
    chord_vector = vector_tools.rotate_vector(np.array([1, 0]), -angle)
    lift, drag, resultant_force, chord_projection_of_force = calculate_lift_and_drag(chord_vector, wind_vector)
    magnitude = vector_tools.calculate_magnitude_of_vector(chord_projection_of_force)
    chord_projection_magnitudes.append(magnitude)

ax2.plot(angles, chord_projection_magnitudes, 'bo-', linewidth=2, markersize=8)
ax2.set_xlabel('Chord Angle (degrees)')
ax2.set_ylabel('Chord Projection Force Magnitude')
ax2.set_title('Chord Projection Force vs Chord Angle')
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, 360)

plt.tight_layout()
plt.show()

# Print numerical results
print("Angle (deg) | Chord Projection Force Magnitude")
print("-" * 45)
for angle, magnitude in zip(angles, chord_projection_magnitudes):
    print(f"{angle:10.0f} | {magnitude:25.6f}")
