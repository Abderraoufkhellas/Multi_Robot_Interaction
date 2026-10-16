from typing import List, Dict, Set, Optional
from .robot import Robot
from .state import Twist
import math

class World:
    """
    The simulation engine 
    """
    def __init__(self, dt: float = 0.1):
        # A list to hold any object that inherits from the abstract Robot class
        self.robots: List[Robot] = []
        self.time: float = 0.0
        self.dt: float = dt

        # Tracking collisions for performance metrics
        self.crashed_robots: Set[int] = set()
        self.collision_event_count: int = 0

    def add_robot(self, robot: Robot):
        """Registers a new robot into the simulation."""
        self.robots.append(robot)


    def _check_collisions(self):
        """Internal method to verify if any physical footprints overlap."""
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                r1 = self.robots[i]
                r2 = self.robots[j]
                
                # If BOTH robots are already frozen from a previous crash, skip them
                if r1.id in self.crashed_robots and r2.id in self.crashed_robots:
                    continue
                
                # Calculate Euclidean distance between centers
                dx = r1.state.x - r2.state.x
                dy = r1.state.y - r2.state.y
                distance = math.hypot(dx, dy)
                
                # Minimum safe distance is the sum of their physical boundaries
                safe_distance = r1.radius + r2.radius
                
                if distance < safe_distance:
                    print(f"[CRASH] Time: {self.time:.1f}s | Robot {r1.id} collided with Robot {r2.id}!")
                    self.collision_event_count += 1
                    self.crashed_robots.add(r1.id)
                    self.crashed_robots.add(r2.id)

    def step(self, commands: Dict[int, Twist]):
        
        for robot in self.robots:
            
            # FREEZE PENALTY: Intercept commands for crashed robots
            if robot.id in self.crashed_robots:
                cmd = Twist(linear_x=0.0, angular_z=0.0)
            else:
                cmd = commands.get(robot.id, Twist(linear_x=0.0, angular_z=0.0))
                
            robot.step(cmd, self.dt)
            
        # Check for new overlaps after movement
        self._check_collisions()
            
        self.time += self.dt