import math
from abc import ABC, abstractmethod
from .state import Twist, Odometry

class Robot(ABC):
    """
    Abstract base class representing a generic robot.
    Holds common properties shared by all robot types (ID, state, physical radius).
    """
    def __init__(self, robot_id: int, start_x: float, start_y: float, start_theta: float, radius: float = 0.15):
        self.id = robot_id
        self.state = Odometry(x=start_x, y=start_y, theta=start_theta)
        
        # Used later for collision detection (e.g., ORCA safety bounds)
        self.radius = radius 

    @abstractmethod
    def step(self, cmd: Twist, dt: float):
        """
        Advances the robot's physical position.
        Every specific robot hardware type MUST implement its own kinematics here.
        """
        pass

class DifferentialRobot(Robot):
    """
    Concrete implementation of a two-wheeled differential drive robot.
    """
    def __init__(self, robot_id: int, start_x: float, start_y: float, start_theta: float, 
                 radius: float = 0.15, track_width: float = 0.3, max_wheel_speed: float = 1.0):
        
        # 1. Call the parent constructor to initialize id, state, and radius
        super().__init__(robot_id, start_x, start_y, start_theta, radius)
        
        # 2. Add properties specific to differential drive hardware
        self.track_width = track_width
        self.max_wheel_speed = max_wheel_speed

    # 3. Override the abstract step method with specific differential kinematics
    def step(self, cmd: Twist, dt: float):
        # Inverse Kinematics
        v_right = cmd.linear_x + (cmd.angular_z * self.track_width / 2.0)
        v_left = cmd.linear_x - (cmd.angular_z * self.track_width / 2.0)
        
        # Hardware Constraints
        v_right = max(-self.max_wheel_speed, min(self.max_wheel_speed, v_right))
        v_left = max(-self.max_wheel_speed, min(self.max_wheel_speed, v_left))
        
        # Forward Kinematics
        actual_v = (v_right + v_left) / 2.0
        actual_w = (v_right - v_left) / self.track_width
        
        # Euler Integration
        self.state.x += actual_v * math.cos(self.state.theta) * dt
        self.state.y += actual_v * math.sin(self.state.theta) * dt
        self.state.theta += actual_w * dt
        
        # Update state telemetry
        self.state.v = actual_v
        self.state.w = actual_w