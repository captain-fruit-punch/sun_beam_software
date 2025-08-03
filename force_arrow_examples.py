from force_arrow import ForceArrowWindow, Body
import math

def example_1_simple_forces():
    """Example 1: Simple forces on a single body."""
    demo = ForceArrowWindow()
    
    # Add a body
    body = demo.add_body(0, 0, 1.0, 'blue', "Mass")
    
    # Add forces
    demo.add_force(body, 3, 0, 'red', 1.0, "F_x")
    demo.add_force(body, 0, 2, 'green', 1.0, "F_y")
    demo.add_force(body, -1, -1, 'orange', 1.0, "F_friction")
    
    # Show total force
    demo.show_total_forces(1.0)
    
    print("Example 1: Simple forces on a single body")
    return demo.app.exec_()

def example_2_equilibrium():
    """Example 2: Forces in equilibrium."""
    demo = ForceArrowWindow()
    
    # Add a body
    body = demo.add_body(0, 0, 1.0, 'purple', "Equilibrium")
    
    # Add balanced forces
    demo.add_force(body, 2, 0, 'red', 1.0, "F1")
    demo.add_force(body, -2, 0, 'red', 1.0, "F2")
    demo.add_force(body, 0, 1.5, 'green', 1.0, "F3")
    demo.add_force(body, 0, -1.5, 'green', 1.0, "F4")
    
    # Show total force (should be close to zero)
    demo.show_total_forces(1.0)
    
    print("Example 2: Forces in equilibrium")
    return demo.app.exec_()

def example_3_multiple_bodies():
    """Example 3: Multiple bodies with interaction forces."""
    demo = ForceArrowWindow()
    
    # Add bodies
    body1 = demo.add_body(-3, 0, 0.8, 'blue', "Body 1")
    body2 = demo.add_body(3, 0, 0.8, 'red', "Body 2")
    body3 = demo.add_body(0, 2, 0.6, 'green', "Body 3")
    
    # Add gravitational forces (simplified)
    demo.add_force(body1, 0, -2, 'brown', 1.0, "Gravity")
    demo.add_force(body2, 0, -2, 'brown', 1.0, "Gravity")
    demo.add_force(body3, 0, -2, 'brown', 1.0, "Gravity")
    
    # Add interaction forces
    demo.add_force(body1, 1, 0, 'orange', 1.0, "Repulsion")
    demo.add_force(body2, -1, 0, 'orange', 1.0, "Repulsion")
    
    # Add tension forces
    demo.add_force(body1, 0.5, 1, 'purple', 1.0, "Tension")
    demo.add_force(body2, -0.5, 1, 'purple', 1.0, "Tension")
    demo.add_force(body3, 0, -1, 'purple', 1.0, "Tension")
    
    # Show total forces
    demo.show_total_forces(1.0)
    
    print("Example 3: Multiple bodies with interaction forces")
    return demo.app.exec_()

def example_4_circular_motion():
    """Example 4: Circular motion forces."""
    demo = ForceArrowWindow()
    
    # Add a body
    body = demo.add_body(0, 0, 0.8, 'blue', "Particle")
    
    # Add centripetal force
    demo.add_force(body, -2, 0, 'red', 1.0, "Centripetal")
    
    # Add tangential velocity (shown as a force for visualization)
    demo.add_force(body, 0, 3, 'green', 1.0, "Velocity")
    
    # Add some damping
    demo.add_force(body, 0.5, -0.5, 'orange', 1.0, "Damping")
    
    # Show total force
    demo.show_total_forces(1.0)
    
    print("Example 4: Circular motion forces")
    return demo.app.exec_()

def example_5_spring_system():
    """Example 5: Spring-mass system."""
    demo = ForceArrowWindow()
    
    # Add bodies representing masses
    mass1 = demo.add_body(-2, 0, 0.6, 'blue', "Mass 1")
    mass2 = demo.add_body(2, 0, 0.6, 'red', "Mass 2")
    
    # Spring forces (simplified Hooke's law)
    # Spring connecting the masses
    demo.add_force(mass1, 1, 0, 'green', 1.0, "Spring")
    demo.add_force(mass2, -1, 0, 'green', 1.0, "Spring")
    
    # Damping forces
    demo.add_force(mass1, -0.3, 0, 'orange', 1.0, "Damping")
    demo.add_force(mass2, 0.3, 0, 'orange', 1.0, "Damping")
    
    # External forces
    demo.add_force(mass1, 0.5, 0, 'purple', 1.0, "External")
    
    # Show total forces
    demo.show_total_forces(1.0)
    
    print("Example 5: Spring-mass system")
    return demo.app.exec_()

def example_6_force_analysis():
    """Example 6: Detailed force analysis with multiple components."""
    demo = ForceArrowWindow()
    
    # Add a body
    body = demo.add_body(0, 0, 1.0, 'blue', "Object")
    
    # Add various force components
    demo.add_force(body, 2.5, 0, 'red', 1.0, "Applied")
    demo.add_force(body, -1.0, 0, 'orange', 1.0, "Friction")
    demo.add_force(body, 0, -1.5, 'brown', 1.0, "Gravity")
    demo.add_force(body, 0, 1.5, 'green', 1.0, "Normal")
    demo.add_force(body, -0.5, -0.3, 'purple', 1.0, "Air Resistance")
    
    # Show total force
    demo.show_total_forces(1.0)
    
    print("Example 6: Detailed force analysis")
    return demo.app.exec_()

def example_7_static_equilibrium():
    """Example 7: Static equilibrium with multiple support forces."""
    demo = ForceArrowWindow()
    
    # Add a body
    body = demo.add_body(0, 0, 1.0, 'blue', "Beam")
    
    # Add support forces (truss-like structure)
    demo.add_force(body, -1.5, 1.0, 'red', 1.0, "Support A")
    demo.add_force(body, 1.5, 1.0, 'red', 1.0, "Support B")
    demo.add_force(body, 0, -2.0, 'brown', 1.0, "Weight")
    
    # Show total force
    demo.show_total_forces(1.0)
    
    print("Example 7: Static equilibrium")
    return demo.app.exec_()

def example_8_vector_decomposition():
    """Example 8: Vector decomposition example."""
    demo = ForceArrowWindow()
    
    # Add a body
    body = demo.add_body(0, 0, 1.0, 'blue', "Particle")
    
    # Add a resultant force
    demo.add_force(body, 3, 2, 'black', 1.0, "Resultant")
    
    # Add its components
    demo.add_force(body, 3, 0, 'red', 1.0, "X Component")
    demo.add_force(body, 0, 2, 'green', 1.0, "Y Component")
    
    # Show total force
    demo.show_total_forces(1.0)
    
    print("Example 8: Vector decomposition")
    return demo.app.exec_()

def interactive_demo():
    """Interactive demo where you can add bodies and forces programmatically."""
    demo = ForceArrowWindow()
    
    print("Interactive Demo")
    print("Available commands:")
    print("1. add_body(x, y, radius, color, label)")
    print("2. add_force(body, fx, fy, color, scale, label)")
    print("3. show_total_forces(scale)")
    print("4. clear_forces()")
    print("5. exit")
    
    # Pre-populate with some example
    body1 = demo.add_body(0, 0, 0.8, 'blue', "Example")
    demo.add_force(body1, 2, 1, 'red', 1.0, "F1")
    demo.add_force(body1, -1, 2, 'green', 1.0, "F2")
    demo.show_total_forces(1.0)
    
    return demo.app.exec_()

if __name__ == "__main__":
    print("Force Arrow Demo Examples")
    print("Choose an example:")
    print("1. Simple forces")
    print("2. Equilibrium")
    print("3. Multiple bodies")
    print("4. Circular motion")
    print("5. Spring system")
    print("6. Force analysis")
    print("7. Static equilibrium")
    print("8. Vector decomposition")
    print("9. Interactive demo")
    
    choice = input("Enter your choice (1-9): ")
    
    if choice == "1":
        example_1_simple_forces()
    elif choice == "2":
        example_2_equilibrium()
    elif choice == "3":
        example_3_multiple_bodies()
    elif choice == "4":
        example_4_circular_motion()
    elif choice == "5":
        example_5_spring_system()
    elif choice == "6":
        example_6_force_analysis()
    elif choice == "7":
        example_7_static_equilibrium()
    elif choice == "8":
        example_8_vector_decomposition()
    elif choice == "9":
        interactive_demo()
    else:
        print("Invalid choice. Running simple demo...")
        example_1_simple_forces() 