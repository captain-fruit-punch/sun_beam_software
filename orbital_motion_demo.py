from force_arrow import ForceArrowWindow, Body
import math
import time
from PyQt5.QtCore import QTimer

class OrbitalMotionDemo:
    """Animated demo showing bodies in orbital motion around a central mass."""
    
    def __init__(self):
        self.demo = ForceArrowWindow()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)
        
        # Animation parameters
        self.time = 0.0
        self.dt = 0.05  # Time step
        self.animation_speed = 50  # ms between updates
        
        # Central mass (sun)
        self.sun = None
        self.sun_mass = 10.0
        
        # Orbiting bodies
        self.planets = []
        self.planet_data = [
            {'name': 'Mercury', 'color': 'orange', 'radius': 0.3, 'semi_major': 2.0, 'eccentricity': 0.2, 'period': 2.0},
            {'name': 'Venus', 'color': 'yellow', 'radius': 0.4, 'semi_major': 3.5, 'eccentricity': 0.1, 'period': 3.5},
            {'name': 'Earth', 'color': 'blue', 'radius': 0.5, 'semi_major': 5.0, 'eccentricity': 0.05, 'period': 5.0},
            {'name': 'Mars', 'color': 'red', 'radius': 0.35, 'semi_major': 6.5, 'eccentricity': 0.15, 'period': 6.5}
        ]
        
        # Trajectory tracking
        self.trajectories = []
        self.max_trajectory_points = 200
        
        self.setup_scenario()
        
    def setup_scenario(self):
        """Set up the initial orbital scenario."""
        print("Setting up orbital motion scenario...")
        
        # Add central sun
        self.sun = self.demo.add_body(0, 0, 1.0, 'yellow', "Sun")
        
        # Add planets with initial orbital positions
        for i, planet_info in enumerate(self.planet_data):
            # Calculate initial position based on orbital parameters
            angle = i * math.pi / 2  # Start planets at different angles
            semi_major = planet_info['semi_major']
            eccentricity = planet_info['eccentricity']
            
            # Calculate position using orbital mechanics
            r = semi_major * (1 - eccentricity**2) / (1 + eccentricity * math.cos(angle))
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # Add planet
            planet = self.demo.add_body(x, y, planet_info['radius'], 
                                      planet_info['color'], planet_info['name'])
            self.planets.append(planet)
            
            # Initialize trajectory tracking
            self.trajectories.append([])
            
            # Add initial forces (will be updated during animation)
            self.demo.add_force(planet, 0, 0, 'purple', 1.0, "Gravity")
            self.demo.add_force(planet, 0, 0, 'green', 1.0, "Velocity")
        
        # Show initial total forces
        self.demo.show_total_forces(1.0)
        
    def calculate_orbital_position(self, semi_major, eccentricity, period, time_offset):
        """Calculate orbital position using Kepler's laws."""
        # Mean anomaly
        mean_anomaly = (2 * math.pi * (self.time + time_offset)) / period
        
        # Solve Kepler's equation for eccentric anomaly (simplified)
        eccentric_anomaly = mean_anomaly  # Approximation for small eccentricity
        
        # Calculate true anomaly
        true_anomaly = 2 * math.atan(math.sqrt((1 + eccentricity) / (1 - eccentricity)) * 
                                   math.tan(eccentric_anomaly / 2))
        
        # Calculate radius
        r = semi_major * (1 - eccentricity**2) / (1 + eccentricity * math.cos(true_anomaly))
        
        # Calculate position
        x = r * math.cos(true_anomaly)
        y = r * math.sin(true_anomaly)
        
        return x, y, true_anomaly
    
    def calculate_velocity(self, semi_major, eccentricity, period, true_anomaly):
        """Calculate orbital velocity."""
        # Orbital velocity magnitude (simplified)
        v_mag = math.sqrt(2 * math.pi * semi_major / period)
        
        # Velocity direction (tangential to orbit)
        vx = -v_mag * math.sin(true_anomaly)
        vy = v_mag * math.cos(true_anomaly)
        
        return vx, vy
    
    def calculate_gravitational_force(self, planet_x, planet_y, planet_mass=1.0):
        """Calculate gravitational force from sun to planet."""
        G = 1.0  # Gravitational constant (scaled)
        distance = math.sqrt(planet_x**2 + planet_y**2)
        
        if distance < 0.1:  # Avoid division by zero
            return 0, 0
        
        # Force magnitude
        force_mag = G * self.sun_mass * planet_mass / (distance**2)
        
        # Force direction (toward sun)
        force_x = -force_mag * planet_x / distance
        force_y = -force_mag * planet_y / distance
        
        return force_x, force_y
    
    def update_animation(self):
        """Update the animation for one time step."""
        self.time += self.dt
        
        # Update each planet
        for i, planet in enumerate(self.planets):
            planet_info = self.planet_data[i]
            
            # Calculate new orbital position
            x, y, true_anomaly = self.calculate_orbital_position(
                planet_info['semi_major'],
                planet_info['eccentricity'],
                planet_info['period'],
                i * 0.5  # Time offset for different starting positions
            )
            
            # Update planet position
            self.demo.update_body_position(i + 1, x, y)  # +1 because sun is at index 0
            
            # Update trajectory
            self.trajectories[i].append((x, y))
            if len(self.trajectories[i]) > self.max_trajectory_points:
                self.trajectories[i].pop(0)
            
            # Calculate and update forces
            gravity_fx, gravity_fy = self.calculate_gravitational_force(x, y)
            vx, vy = self.calculate_velocity(
                planet_info['semi_major'],
                planet_info['eccentricity'],
                planet_info['period'],
                true_anomaly
            )
            
            # Update force arrows
            if len(planet.forces) >= 2:
                planet.forces[0].update_force(gravity_fx, gravity_fy)  # Gravity
                planet.forces[1].update_force(vx, vy)  # Velocity
        
        # Redraw forces
        self.demo.redraw_forces()
        
        # Draw trajectories
        self.draw_trajectories()
    
    def draw_trajectories(self):
        """Draw orbital trajectories."""
        # Clear previous trajectories (this is a simplified approach)
        # In a more sophisticated implementation, you'd track trajectory items
        
        # For now, we'll just update the plot range to follow the planets
        if self.planets:
            x_coords = [planet.x for planet in self.planets]
            y_coords = [planet.y for planet in self.planets]
            
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            
            # Add some padding
            padding = 2.0
            self.demo.plot.setXRange(x_min - padding, x_max + padding)
            self.demo.plot.setYRange(y_min - padding, y_max + padding)
    
    def start_animation(self):
        """Start the orbital animation."""
        print("Starting orbital motion animation...")
        print("Close the window to stop the animation.")
        
        # Start the timer
        self.timer.start(self.animation_speed)
        
        # Run the application
        return self.demo.app.exec_()
    
    def stop_animation(self):
        """Stop the animation."""
        self.timer.stop()

def create_orbital_demo():
    """Create and run the orbital motion demo."""
    demo = OrbitalMotionDemo()
    return demo.start_animation()

if __name__ == "__main__":
    create_orbital_demo() 