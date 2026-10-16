from typing import List, Dict, Set, Optional, Tuple
from .robot import Robot
from .state import Twist, Odometry
import math
from .obstacle import CircularObstacle


class World:
    """
    The simulation engine 
    """
    def __init__(self, dt: float = 0.1):
        # A list to hold any object that inherits from the abstract Robot class
        self.robots: List[Robot] = []
        self.time: float = 0.0
        self.dt: float = dt
        self.obstacles: List[CircularObstacle] = []

        # Tracking collisions for performance metrics
        self.crashed_robots: Set[int] = set()
        self.collision_event_count: int = 0

    def add_robot(self, robot: Robot):
        """Registers a new robot into the simulation."""
        self.robots.append(robot)

    def get_robot_by_id(self, robot_id: int) -> Optional[Robot]:
        """Helper method to find a robot in the world."""
        for robot in self.robots:
            if robot.id == robot_id:
                return robot
        return None
    
    def add_obstacle(self, obstacle: CircularObstacle):
        self.obstacles.append(obstacle)

    def get_lidar_scan(self, querying_robot_id: int, max_range: float = 3.0) -> Tuple[List[Odometry], List[Tuple[float, float]]]:
        """
        Returns a tuple of (dynamic_neighbors, static_points).
        - dynamic_neighbors: Odometry of moving robots.
        - static_points: (x, y) coordinates of the closest edges of static obstacles.
        """
        detected_neighbors = []
        static_points = []
        
        querying_robot = self.get_robot_by_id(querying_robot_id)
        if not querying_robot:
            return detected_neighbors, static_points

        # 1. Scan for dynamic neighbors
        for other_robot in self.robots:
            if other_robot.id == querying_robot_id:
                continue
            
            dx = other_robot.state.x - querying_robot.state.x
            dy = other_robot.state.y - querying_robot.state.y
            distance = math.hypot(dx, dy)
            
            if distance <= max_range:
                detected_neighbors.append(other_robot.state)
                
        # 2. Scan for static obstacles
        for obs in self.obstacles:
            dx = obs.x - querying_robot.state.x
            dy = obs.y - querying_robot.state.y
            distance_to_center = math.hypot(dx, dy)
            
            # Check if the closest edge of the obstacle is within LiDAR range
            if distance_to_center - obs.radius <= max_range:
                # Calculate the exact (x, y) coordinate on the obstacle's surface closest to the robot
                if distance_to_center > 0:
                    ux = dx / distance_to_center
                    uy = dy / distance_to_center
                    closest_x = obs.x - (ux * obs.radius)
                    closest_y = obs.y - (uy * obs.radius)
                    static_points.append((closest_x, closest_y))
                
        return detected_neighbors, static_points

    def _check_collisions(self):
        """Checks for overlaps between robots, and between robots and obstacles."""
        # 1. Robot-to-Robot collisions
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                r1 = self.robots[i]
                r2 = self.robots[j]
                
                if r1.id in self.crashed_robots and r2.id in self.crashed_robots:
                    continue
                
                distance = math.hypot(r1.state.x - r2.state.x, r1.state.y - r2.state.y)
                if distance < (r1.radius + r2.radius):
                    print(f"[CRASH] Time: {self.time:.1f}s | Robot {r1.id} collided with Robot {r2.id}!")
                    self.collision_event_count += 1
                    self.crashed_robots.add(r1.id)
                    self.crashed_robots.add(r2.id)
                    
        # 2. Robot-to-Obstacle collisions
        for r in self.robots:
            if r.id in self.crashed_robots:
                continue
                
            for obs in self.obstacles:
                distance = math.hypot(r.state.x - obs.x, r.state.y - obs.y)
                if distance < (r.radius + obs.radius):
                    print(f"[CRASH] Time: {self.time:.1f}s | Robot {r.id} hit a static obstacle!")
                    self.collision_event_count += 1
                    self.crashed_robots.add(r.id)
                    break # Skip checking other obstacles for this robot if it already crashed

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