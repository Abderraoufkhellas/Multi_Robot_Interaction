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
        
        # Physical hardware limits
        self.track_width = 0.20  # Distance between wheels (m)
        self.max_wheel_speed = 0.5 # Maximum physical surface speed of the wheels (m/s)
        
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0

    def plan(self, target_x: float, target_y: float, dynamic_neighbors: list, static_points: list):
        if not self.global_planner or not self.local_planner:
            return

        pref_vx, pref_vy = self.global_planner.compute_preferred_velocity(
            self.state, target_x, target_y
        )
        
        self.cmd = self.local_planner.compute_safe_velocity(
            self.state, pref_vx, pref_vy, dynamic_neighbors, static_points
        )

    def step(self, dt: float):
        # 1. Inverse Kinematics: Convert Twist to ideal wheel surface speeds
        v_r_ideal = self.cmd.linear_x + (self.cmd.angular_z * self.track_width / 2.0)
        v_l_ideal = self.cmd.linear_x - (self.cmd.angular_z * self.track_width / 2.0)
        
        # 2. Motor Saturation (Proportional Scaling)
        max_calculated = max(abs(v_r_ideal), abs(v_l_ideal))
        
        if max_calculated > self.max_wheel_speed:
            # Scale both wheels down by the exact same ratio to preserve the turning arc
            scale_factor = self.max_wheel_speed / max_calculated
            self.wheel_speed_right = v_r_ideal * scale_factor
            self.wheel_speed_left = v_l_ideal * scale_factor
        else:
            self.wheel_speed_right = v_r_ideal
            self.wheel_speed_left = v_l_ideal

        # 3. Forward Kinematics: The physical world applies clamped speeds to the chassis
        actual_v = (self.wheel_speed_right + self.wheel_speed_left) / 2.0
        actual_w = (self.wheel_speed_right - self.wheel_speed_left) / self.track_width

        # 4. Update the state
        self.state.x += actual_v * math.cos(self.state.theta) * dt
        self.state.y += actual_v * math.sin(self.state.theta) * dt
        self.state.theta += actual_w * dt
        
        self.state.v = actual_v
        self.state.w = actual_w