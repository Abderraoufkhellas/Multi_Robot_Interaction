import json
from typing import Tuple, Dict, Any
from src.simulator.world import World
from src.simulator.robot import DifferentialRobot
from src.simulator.obstacle import CircularObstacle
from src.planners.go_to_goal import GoToGoalPlanner
from src.planners.orca_planner import OrcaPlanner

ROBOT_CLASSES = {"DifferentialRobot": DifferentialRobot}
PLANNER_CLASSES = {
    "GoToGoalPlanner": GoToGoalPlanner,
    "OrcaPlanner": OrcaPlanner
}

def setup_simulation(filepath: str) -> Tuple[World, Dict[int, Tuple[float, float]], dict]:
    with open(filepath, 'r') as f:
        data = json.load(f)
        
    sim_config = data.get("simulation", {"dt": 0.1, "max_time": 20.0})
    world = World(dt=sim_config["dt"])
    
    for obs in data.get('obstacles', []):
        world.add_obstacle(CircularObstacle(x=obs['x'], y=obs['y'], radius=obs['radius']))
        
    targets = {}
    
    for r_data in data.get('robots', []):
        r_id = r_data['id']
        
        # 1. Instantiate the physical robot
        robot_class = ROBOT_CLASSES[r_data.get('type', 'DifferentialRobot')]
        robot = robot_class(
            robot_id=r_id,
            start_x=r_data['start']['x'],
            start_y=r_data['start']['y'],
            start_theta=r_data['start']['theta'],
            radius=r_data.get('radius', 0.14)
        )
        
        # 2. Instantiate the navigation stack
        nav_config = r_data.get('navigation', {})
        global_class = PLANNER_CLASSES[nav_config.get('global', 'GoToGoalPlanner')]
        local_class = PLANNER_CLASSES[nav_config.get('local', 'OrcaPlanner')]
        
        # 3. Mount the stack onto the robot
        robot.set_planners(
            global_planner=global_class(robot_id=r_id),
            local_planner=local_class(robot_id=r_id, dt=sim_config["dt"])
        )
        
        world.add_robot(robot)
        targets[r_id] = (r_data['target']['x'], r_data['target']['y'])
        
    return world, targets, sim_config