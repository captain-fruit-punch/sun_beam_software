import numpy as np
import matplotlib.pyplot as plt
from pandas import read_csv
import parse_foil_data
import vector_tools
from matplotlib.animation import FuncAnimation

foil_data = read_csv('xf-ag03-il-50000.csv', skiprows=10)

blade_height = 1
chord_vector = vector_tools.rotate_vector(np.array([1, 0]), -5) # points towards the trailing edge
wind_vector = np.array([1, 0])
air_density = 1.225

def calculate_lift_and_drag(chord_vector, wind_vector):
    aoa = vector_tools.calculate_angle_between_vectors(chord_vector, wind_vector)

    drag_unit_vector = vector_tools.calculate_unit_vector(wind_vector)
    lift_unit_vector = vector_tools.calculate_unit_vector(vector_tools.rotate_vector(wind_vector, 90))

    cl_at_aoa = parse_foil_data.get_c_l_from_foil_data(foil_data, aoa)
    cd_at_aoa = parse_foil_data.get_c_d_from_foil_data(foil_data, aoa)

    area_of_foil = blade_height * vector_tools.calculate_magnitude_of_vector(chord_vector)

    wind_speed = vector_tools.calculate_magnitude_of_vector(wind_vector)

    lift_magnitude = cl_at_aoa * air_density * wind_speed**2 * area_of_foil / 2
    drag_magnitude = cd_at_aoa * air_density * wind_speed**2 * area_of_foil / 2

    lift = lift_magnitude * lift_unit_vector
    drag = drag_magnitude * drag_unit_vector
    resultant_force = lift + drag

    chord_projection_of_force = vector_tools.calculate_projection(resultant_force, chord_vector) * vector_tools.calculate_unit_vector(chord_vector)
    return lift, drag, resultant_force, chord_projection_of_force

# Animation function
def animate_rotation(frame):
    # Clear the previous frame
    plt.clf()
    
    # Calculate rotation angle (0 to 360 degrees)
    angle = frame * 360 / 100  # 100 frames for full rotation
    
    # Rotate the chord vector
    rotated_chord = vector_tools.rotate_vector(np.array([1, 0]), angle - 5)  # -5 for initial offset
    
    # Calculate forces for this rotation
    lift, drag, resultant_force, chord_projection_of_force = calculate_lift_and_drag(rotated_chord, wind_vector)
    
    origin = np.array([0, 0])
    
    # Plot chord vector
    plt.quiver(*origin, *rotated_chord, color='b', angles='xy', scale_units='xy', scale=1, label='Chord Vector')
    
    # Plot wind vector
    plt.quiver(*origin, *wind_vector, color='g', angles='xy', scale_units='xy', scale=1, label='Wind Vector')
    
    # Plot lift vector
    plt.quiver(*origin, *lift, color='r', angles='xy', scale_units='xy', scale=1, label='Lift')
    
    # Plot drag vector
    plt.quiver(*origin, *drag, color='m', angles='xy', scale_units='xy', scale=1, label='Drag')
    
    # Plot resultant force
    plt.quiver(*origin, *resultant_force, color='y', angles='xy', scale_units='xy', scale=1, label='Resultant Force')
    
    # Plot chord projection of force
    plt.quiver(*origin, *chord_projection_of_force, color='c', angles='xy', scale_units='xy', scale=1, label='Chord Projection of Force')
    
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.grid(True)
    plt.legend()
    plt.title(f'VAWT Blade Rotation - Angle: {angle:.1f}°')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.gca().set_aspect('equal', adjustable='box')

# Create the animation
fig, ax = plt.subplots(figsize=(10, 10))
ani = FuncAnimation(fig, animate_rotation, frames=100, interval=100, repeat=True)

plt.show()

# Keep the original static plot for reference
lift, drag, resultant_force, chord_projection_of_force = calculate_lift_and_drag(chord_vector, wind_vector)
print(lift)
print(drag)
print(resultant_force)
print(chord_projection_of_force)

plt.figure(figsize=(7,7))
origin = np.array([0, 0])

# Plot chord vector
plt.quiver(*origin, *chord_vector, color='b', angles='xy', scale_units='xy', scale=1, label='Chord Vector')

# Plot wind vector
plt.quiver(*origin, *wind_vector, color='g', angles='xy', scale_units='xy', scale=1, label='Wind Vector')

# Plot lift vector
plt.quiver(*origin, *lift, color='r', angles='xy', scale_units='xy', scale=1, label='Lift')

# Plot drag vector
plt.quiver(*origin, *drag, color='m', angles='xy', scale_units='xy', scale=1, label='Drag')

# Plot resultant force
plt.quiver(*origin, *resultant_force, color='y', angles='xy', scale_units='xy', scale=1, label='Resultant Force')

# Plot chord projection of force
plt.quiver(*origin, *chord_projection_of_force, color='c', angles='xy', scale_units='xy', scale=1, label='Chord Projection of Force')

plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.grid(True)
plt.legend()
plt.title('Visualization of Lift, Drag, Chord, and Wind Vectors')
plt.xlabel('X')
plt.ylabel('Y')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
