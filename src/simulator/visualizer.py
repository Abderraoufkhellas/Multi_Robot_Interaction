import matplotlib.pyplot as plt
import numpy as np
from .world import World

class Visualizer:
    """
    Renders the 2D simulation state.
    """
    def __init__(self, world: World, bounds: float = 5.0):
        self.world = world
        self.bounds = bounds
        
        # plt.ion() turns on interactive mode 
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def render(self):
        """Clears the previous frame and redraws the current state of the world."""
        self.ax.clear()
        
        # Keep the grid fixed
        self.ax.set_xlim(-self.bounds, self.bounds)
        self.ax.set_ylim(-self.bounds, self.bounds)
        self.ax.set_aspect('equal') 
        self.ax.set_title(f"Multi-Robot Simulation | Time: {self.world.time:.2f}s")
        self.ax.grid(True, linestyle='--', alpha=0.6)

        for robot in self.world.robots:
            # Draw the physical footprint 
            circle = plt.Circle((robot.state.x, robot.state.y), robot.radius, 
                                fill=False, color='blue', linewidth=2)
            self.ax.add_patch(circle)
            
            # Draw a line to indicate orientation 
            line_length = robot.radius * 1.5
            end_x = robot.state.x + line_length * np.cos(robot.state.theta)
            end_y = robot.state.y + line_length * np.sin(robot.state.theta)
            self.ax.plot([robot.state.x, end_x], [robot.state.y, end_y], color='red', linewidth=2)
            
            # Add a label for the robot ID
            self.ax.text(robot.state.x, robot.state.y + robot.radius + 0.1, 
                         f"R{robot.id}", ha='center', fontsize=9)

        
        plt.draw()
        
        # Pause briefly to allow the GUI to render. 
        pause_time = max(0.01, self.world.dt)
        plt.pause(pause_time)