from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np
from typing import List, Tuple, Dict, Optional, Callable
import math

class ForceArrow:
    """Represents a single force arrow with position, direction, and magnitude."""
    
    def __init__(self, x: float, y: float, fx: float, fy: float, 
                 color: str = 'red', scale: float = 1.0, label: str = ""):
        self.x = x
        self.y = y
        self.fx = fx
        self.fy = fy
        self.color = color
        self.scale = scale
        self.label = label
        self.arrow_length = math.sqrt(fx**2 + fy**2) * scale
        self.angle = math.atan2(fy, fx)
        
    def get_arrow_points(self) -> Tuple[List[float], List[float]]:
        """Get the points for drawing the arrow."""
        # Arrow head parameters
        head_length = min(self.arrow_length * 0.2, 0.5)
        head_angle = math.pi / 6  # 30 degrees
        
        # Main arrow line
        end_x = self.x + self.fx * self.scale
        end_y = self.y + self.fy * self.scale
        
        # Arrow head points
        head_angle1 = self.angle + head_angle
        head_angle2 = self.angle - head_angle
        
        head1_x = end_x - head_length * math.cos(head_angle1)
        head1_y = end_y - head_length * math.sin(head_angle1)
        head2_x = end_x - head_length * math.cos(head_angle2)
        head2_y = end_y - head_length * math.sin(head_angle2)
        
        return ([self.x, end_x, head1_x, end_x, head2_x], 
                [self.y, end_y, head1_y, end_y, head2_y])
    
    def update_force(self, fx: float, fy: float):
        """Update the force components and recalculate arrow properties."""
        self.fx = fx
        self.fy = fy
        self.arrow_length = math.sqrt(fx**2 + fy**2) * self.scale
        self.angle = math.atan2(fy, fx)
    
    def update_position(self, x: float, y: float):
        """Update the position of the force arrow."""
        self.x = x
        self.y = y

class Body:
    """Represents a body with position and associated forces."""
    
    def __init__(self, x: float, y: float, radius: float = 0.5, 
                 color: str = 'blue', label: str = ""):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.label = label
        self.forces: List[ForceArrow] = []
        
    def add_force(self, fx: float, fy: float, color: str = 'red', 
                  scale: float = 1.0, label: str = "") -> ForceArrow:
        """Add a force to this body."""
        force = ForceArrow(self.x, self.y, fx, fy, color, scale, label)
        self.forces.append(force)
        return force
        
    def get_total_force(self) -> Tuple[float, float]:
        """Calculate the total force on this body."""
        total_fx = sum(f.fx for f in self.forces)
        total_fy = sum(f.fy for f in self.forces)
        return total_fx, total_fy
    
    def update_position(self, x: float, y: float):
        """Update the position of the body and all its forces."""
        self.x = x
        self.y = y
        for force in self.forces:
            force.update_position(x, y)

class ForceArrowWindow:
    """Main demo class for displaying bodies and forces with arrows."""
    
    def __init__(self):
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Force Arrow Demo")
        self.win.resize(800, 600)
        self.win.show()
        
        # Create the main plot
        self.plot = self.win.addPlot(title="Bodies and Forces")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        
        # Set up the coordinate system
        self.plot.setXRange(-10, 10)
        self.plot.setYRange(-10, 10)
        
        # Add axis labels
        self.plot.setLabel('left', 'Y Position')
        self.plot.setLabel('bottom', 'X Position')
        
        # Store bodies and their visual elements
        self.bodies: List[Body] = []
        self.body_circles: List[pg.ScatterPlotItem] = []
        self.force_arrows: List[pg.PlotDataItem] = []
        self.total_force_arrows: List[pg.PlotDataItem] = []
        self.labels: List[pg.TextItem] = []
        
        # Color palette for different force types
        self.force_colors = ['red', 'green', 'orange', 'purple', 'brown', 'pink']
        
    def add_body(self, x: float, y: float, radius: float = 0.5, 
                 color: str = 'blue', label: str = "") -> Body:
        """Add a body to the simulation."""
        body = Body(x, y, radius, color, label)
        self.bodies.append(body)
        
        # Create visual representation
        circle = pg.ScatterPlotItem(x=[x], y=[y], size=radius*20, 
                                   brush=color, pen=None)
        self.plot.addItem(circle)
        self.body_circles.append(circle)
        
        # Add label if provided
        if label:
            text = pg.TextItem(text=label, color=color)
            text.setPos(x + radius + 0.2, y)
            self.plot.addItem(text)
            self.labels.append(text)
            
        return body
    
    def add_force(self, body: Body, fx: float, fy: float, 
                  color: Optional[str] = None, scale: float = 1.0, 
                  label: str = "") -> ForceArrow:
        """Add a force to a body."""
        if color is None:
            color = self.force_colors[len(body.forces) % len(self.force_colors)]
            
        force = body.add_force(fx, fy, color, scale, label)
        self._draw_force_arrow(force)
        return force
    
    def _draw_force_arrow(self, force: ForceArrow):
        """Draw a force arrow on the plot."""
        x_points, y_points = force.get_arrow_points()
        
        # Draw the main arrow line
        arrow_line = pg.PlotDataItem(x=x_points[:2], y=y_points[:2], 
                                   pen=pg.mkPen(color=force.color, width=2))
        self.plot.addItem(arrow_line)
        self.force_arrows.append(arrow_line)
        
        # Draw the arrow head
        arrow_head = pg.PlotDataItem(x=x_points[1:], y=y_points[1:], 
                                   pen=pg.mkPen(color=force.color, width=2))
        self.plot.addItem(arrow_head)
        self.force_arrows.append(arrow_head)
        
        # Add force label if provided
        if force.label:
            text = pg.TextItem(text=force.label, color=force.color)
            text.setPos(force.x + force.fx * force.scale * 0.5, 
                       force.y + force.fy * force.scale * 0.5)
            self.plot.addItem(text)
            self.labels.append(text)
    
    def show_total_forces(self, scale: float = 1.0):
        """Show the total force on each body as a thick arrow."""
        for body in self.bodies:
            total_fx, total_fy = body.get_total_force()
            if abs(total_fx) > 0.01 or abs(total_fy) > 0.01:  # Only show if significant
                total_force = ForceArrow(body.x, body.y, total_fx, total_fy, 
                                       'black', scale, "Total")
                x_points, y_points = total_force.get_arrow_points()
                
                # Draw total force as a thick arrow
                total_arrow = pg.PlotDataItem(x=x_points[:2], y=y_points[:2], 
                                            pen=pg.mkPen(color='black', width=4))
                self.plot.addItem(total_arrow)
                self.total_force_arrows.append(total_arrow)
                
                # Add "Total" label
                text = pg.TextItem(text="Total", color='black')
                text.setPos(body.x + total_fx * scale * 0.6, 
                           body.y + total_fy * scale * 0.6)
                self.plot.addItem(text)
                self.labels.append(text)
    
    def clear_forces(self):
        """Clear all force arrows."""
        for arrow in self.force_arrows:
            self.plot.removeItem(arrow)
        self.force_arrows.clear()
        
        for arrow in self.total_force_arrows:
            self.plot.removeItem(arrow)
        self.total_force_arrows.clear()
        
        for label in self.labels:
            self.plot.removeItem(label)
        self.labels.clear()
        
        # Clear forces from bodies
        for body in self.bodies:
            body.forces.clear()
    
    def update_body_position(self, body_index: int, x: float, y: float):
        """Update the position of a body and its visual representation."""
        if 0 <= body_index < len(self.bodies):
            body = self.bodies[body_index]
            body.update_position(x, y)
            
            # Update visual representation
            self.body_circles[body_index].setData([x], [y])
            
            # Update label position if it exists
            label_index = body_index
            if label_index < len(self.labels):
                self.labels[label_index].setPos(x + body.radius + 0.2, y)

    def redraw_forces(self):
        """Redraw all forces after updates."""
        self.clear_forces()
        
        # Redraw all forces from all bodies
        for body in self.bodies:
            for force in body.forces:
                self._draw_force_arrow(force)
        
        # Show total forces
        self.show_total_forces(1.0)
    
    def run_demo(self):
        """Run the interactive demo."""
        # Example: Create a simple physics scenario
        print("Creating demo scenario...")
        
        # Add some bodies
        body1 = self.add_body(0, 0, 0.8, 'blue', "Body 1")
        body2 = self.add_body(3, 2, 0.6, 'green', "Body 2")
        body3 = self.add_body(-2, 3, 0.7, 'red', "Body 3")
        
        # Add forces to body 1
        self.add_force(body1, 2, 1, 'red', 1.0, "F1")
        self.add_force(body1, -1, 2, 'green', 1.0, "F2")
        
        # Add forces to body 2
        self.add_force(body2, 1.5, -1, 'orange', 1.0, "F3")
        self.add_force(body2, 0, -2, 'purple', 1.0, "F4")
        
        # Add forces to body 3
        self.add_force(body3, -1.5, -1, 'brown', 1.0, "F5")
        self.add_force(body3, 1, 0.5, 'pink', 1.0, "F6")
        
        # Show total forces
        self.show_total_forces(1.0)
        
        print("Demo running! Close the window to exit.")
        return self.app.exec_()

def create_simple_demo():
    """Create and run a simple force arrow demo."""
    demo = ForceArrowWindow()
    return demo.run_demo()

if __name__ == "__main__":
    create_simple_demo()
