from abc import ABC

class BasePlanner(ABC):
    """
    Base class for all navigation algorithms.
    In the decoupled architecture, specific planner types (Global vs Local)
    implement their own specific computation methods rather than a shared interface.
    """
    def __init__(self, robot_id: int):
        self.robot_id = robot_id