import math
from abc import ABC, abstractmethod
from src.simulator.state import Odometry, Twist

class Robot(ABC):
    def __init__(self, robot_id: int, radius: float):
        self.id = robot_id
        self.radius = radius
        self.state = None  # To be initialized by subclass
        
        # Internal navigation stack
        self.global_planner = None
        self.local_planner = None
        self.cmd = Twist(0.0, 0.0)

    def set_planners(self, global_planner, local_planner):
        """Attaches the navigation algorithms to the robot's brain."""
        self.global_planner = global_planner
        self.local_planner = local_planner

    @abstractmethod
    def plan(self, target_x: float, target_y: float, dynamic_neighbors: list, static_points: list):
        """Executes the navigation stack to calculate the next motor command."""
        pass

    @abstractmethod
    def step(self, dt: float):
        """Applies the computed motor command to the physical kinematic model."""
        pass


class DifferentialRobot(Robot):
    def __init__(self, robot_id: int, start_x: float, start_y: float, start_theta: float, radius: float = 0.14):
        super().__init__(robot_id, radius)
        self.state = Odometry(start_x, start_y, start_theta, 0.0, 0.0)

    def plan(self, target_x: float, target_y: float, dynamic_neighbors: list, static_points: list):
        if not self.global_planner or not self.local_planner:
            return

        # 1. Global Planner calculates intent
        pref_vx, pref_vy = self.global_planner.compute_preferred_velocity(
            self.state, target_x, target_y
        )
        
        # 2. Local Planner calculates safety
        self.cmd = self.local_planner.compute_safe_velocity(
            self.state, pref_vx, pref_vy, dynamic_neighbors, static_points
        )

    def step(self, dt: float):
        # Kinematic update for a differential drive robot
        self.state.x += self.cmd.linear_x * math.cos(self.state.theta) * dt
        self.state.y += self.cmd.linear_x * math.sin(self.state.theta) * dt
        self.state.theta += self.cmd.angular_z * dt
        
        self.state.linear_x = self.cmd.linear_x
        self.state.angular_z = self.cmd.angular_z