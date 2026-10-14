from dataclasses import dataclass

@dataclass
class Twist:
    """ROS 2 Twist equivalent (cmd_vel).
    We only need linear X and angular Z for 2D unicycle robots.
    """
    linear_x: float = 0.0
    angular_z: float = 0.0

@dataclass
class Odometry:
    """ROS 2 Odometry equivalent.
    """
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0  # Current linear velocity
    w: float = 0.0  # Current angular velocity