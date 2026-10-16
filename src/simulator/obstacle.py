class CircularObstacle:
    """
    Represents a static physical barrier in the environment.
    """
    def __init__(self, x: float, y: float, radius: float):
        self.x = x
        self.y = y
        self.radius = radius