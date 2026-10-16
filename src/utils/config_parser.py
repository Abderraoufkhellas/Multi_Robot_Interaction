import json
from typing import Tuple, Dict, Any
from src.simulator.world import World
from src.simulator.robot import DifferentialRobot
from src.simulator.obstacle import CircularObstacle
from src.planners.go_to_goal import GoToGoalPlanner

# Explicit registries: Easy to read, easy for your IDE to track
ROBOT_CLASSES = {
    "DifferentialRobot": DifferentialRobot
}

PLANNER_CLASSES = {
    "GoToGoalPlanner": GoToGoalPlanner
}

def setup_simulation(filepath: str) -> Tuple[World, Dict[int, Tuple[float, float]], Dict[int, Any], dict]:
    with open(filepath, 'r') as f:
        data = json.load(f)
        
    sim_config = data.get("simulation", {"dt": 0.1, "max_time": 20.0})
    world = World(dt=sim_config["dt"])
    
    for obs in data.get('obstacles', []):
        world.add_obstacle(CircularObstacle(x=obs['x'], y=obs['y'], radius=obs['radius']))
        
    targets = {}
    planners = {}
    
    for r_data in data.get('robots', []):
        r_id = r_data['id']
        
        robot_class = ROBOT_CLASSES[r_data.get('type', 'DifferentialRobot')]
        robot = robot_class(
            robot_id=r_id,
            start_x=r_data['start']['x'],
            start_y=r_data['start']['y'],
            start_theta=r_data['start']['theta'],
            radius=r_data.get('radius', 0.14)
        )
        world.add_robot(robot)
        
        planner_class = PLANNER_CLASSES[r_data.get('planner', 'GoToGoalPlanner')]
        planners[r_id] = planner_class(robot_id=r_id)
        
        targets[r_id] = (r_data['target']['x'], r_data['target']['y'])
        
    return world, targets, planners, sim_config