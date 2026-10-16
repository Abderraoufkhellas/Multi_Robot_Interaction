from typing import List, Dict
from .robot import Robot
from .state import Twist

class World:
    """
    The simulation engine 
    """
    def __init__(self, dt: float = 0.1):
        # A list to hold any object that inherits from the abstract Robot class
        self.robots: List[Robot] = []
        self.time: float = 0.0
        self.dt: float = dt

    def add_robot(self, robot: Robot):
        """Registers a new robot into the simulation."""
        self.robots.append(robot)

    def step(self, commands: Dict[int, Twist]):
        """
        Advances the global simulation clock by one timestep (dt).
        
        :param commands: A dictionary mapping a robot's ID to its desired Twist maneuver.
                         Example: {1: Twist(linear_x=0.5), 2: Twist(angular_z=-0.2)}
        """
        for robot in self.robots:
            # Look up the command for this specific robot. 
            
            cmd = commands.get(robot.id, Twist(linear_x=0.0, angular_z=0.0))
            
            # Tell the robot to physically move 
            robot.step(cmd, self.dt)
            
        # Advance the global simulation clock
        self.time += self.dt