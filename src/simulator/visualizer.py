import matplotlib.pyplot as plt
import numpy as np
from .world import World
import math

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

    def render(self, targets: dict = None):
        """Clears the previous frame and redraws the current state of the world."""
        self.ax.clear()
        
        self.ax.set_xlim(-self.bounds, self.bounds)
        self.ax.set_ylim(-self.bounds, self.bounds)
        self.ax.set_aspect('equal') 
        self.ax.set_title(f"Multi-Robot Simulation | Time: {self.world.time:.2f}s")
        self.ax.grid(True, linestyle='--', alpha=0.6)

        # Draw static obstacles in gray
        for obs in self.world.obstacles:
            circle = plt.Circle((obs.x, obs.y), obs.radius, color='gray', alpha=0.5)
            self.ax.add_patch(circle)

        # Draw target markers 
        if targets:
            for r_id, (tx, ty) in targets.items():
                self.ax.plot(tx, ty, marker='x', color='green', markersize=10, markeredgewidth=2)
                self.ax.text(tx, ty + 0.15, f"Goal {r_id}", ha='center', fontsize=8, color='green')

        for robot in self.world.robots:
            # Change color if crashed
            color = 'red' if robot.id in self.world.crashed_robots else 'blue'
            
            circle = plt.Circle((robot.state.x, robot.state.y), robot.radius, 
                                fill=False, color=color, linewidth=2)
            self.ax.add_patch(circle)
            
            line_length = robot.radius * 1.5
            end_x = robot.state.x + line_length * math.cos(robot.state.theta)
            end_y = robot.state.y + line_length * math.sin(robot.state.theta)
            self.ax.plot([robot.state.x, end_x], [robot.state.y, end_y], color=color, linewidth=2)
            
            self.ax.text(robot.state.x, robot.state.y + robot.radius + 0.1, 
                         f"R{robot.id}", ha='center', fontsize=9)

        plt.draw()
        pause_time = max(0.01, self.world.dt)
        plt.pause(pause_time)

        
        plt.draw()
        
        # Pause briefly to allow the GUI to render. 
        pause_time = max(0.01, self.world.dt)
        plt.pause(pause_time)