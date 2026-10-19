import math
import rvo2
from src.planners.base_planner import BasePlanner
from src.simulator.state import Odometry, Twist

class OrcaPlanner(BasePlanner):
    def __init__(self, robot_id: int, dt: float = 0.1):
        super().__init__(robot_id)
        self.dt = dt
        self.max_speed = 0.5
        self.radius = 0.14
        self.k_w = 2.0 

    def compute_safe_velocity(self, current_state: Odometry, pref_vx: float, pref_vy: float, 
                              dynamic_neighbors: list, static_points: list) -> Twist:
        
        sim = rvo2.PyRVOSimulator(self.dt, 3.0, 10, 2.0, 2.0, self.radius, self.max_speed)

        # 1. Add Self
        sim.addAgent(
            (current_state.x, current_state.y),
            neighborDist=3.0, maxNeighbors=10, timeHorizon=2.0, timeHorizonObst=2.0,
            radius=self.radius, maxSpeed=self.max_speed,
            velocity=(current_state.v * math.cos(current_state.theta), 
                      current_state.v * math.sin(current_state.theta))
        )

        # 2. Add Neighbors
        for neighbor in dynamic_neighbors:
            sim.addAgent(
                (neighbor.x, neighbor.y),
                neighborDist=3.0, maxNeighbors=10, timeHorizon=2.0, timeHorizonObst=2.0,
                radius=self.radius, maxSpeed=self.max_speed,
                velocity=(neighbor.v * math.cos(neighbor.theta), 
                          neighbor.v * math.sin(neighbor.theta))
            )

        # 3. Add Obstacles
        for pt in static_points:
            x, y = pt[0], pt[1]
            eps = 0.01
            sim.addObstacle([(x - eps, y - eps), (x + eps, y - eps), 
                             (x + eps, y + eps), (x - eps, y + eps)])
        sim.processObstacles()

        # 4. Inject the Preferred Vector from the Global Planner
        sim.setAgentPrefVelocity(0, (pref_vx, pref_vy))

        # 5. Solve & Extract
        sim.doStep()
        safe_vx, safe_vy = sim.getAgentVelocity(0)

        # 6. Convert to Twist (v, w)
        safe_speed = math.hypot(safe_vx, safe_vy)
        if safe_speed < 0.01:
            return Twist(0.0, 0.0)

        safe_heading = math.atan2(safe_vy, safe_vx)
        heading_error = safe_heading - current_state.theta
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        v = max(0.0, safe_speed * math.cos(heading_error))
        w = self.k_w * heading_error

        return Twist(v, w)