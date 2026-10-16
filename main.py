from src.simulator.world import World
from src.simulator.robot import DifferentialRobot
from src.simulator.obstacle import CircularObstacle
from src.simulator.visualizer import Visualizer
from src.planners.go_to_goal import GoToGoalPlanner
import matplotlib.pyplot as plt

def main():
    world = World(dt=0.02)

    # Place an obstacle directly in the center
    world.add_obstacle(CircularObstacle(x=-2.0, y=0.0, radius=0.5))

    # Spawn two robots on opposite sides
    robot1 = DifferentialRobot(robot_id=1, start_x=-2.0, start_y=-2.0, start_theta=0.0, radius=0.14)
    robot2 = DifferentialRobot(robot_id=2, start_x=2.0, start_y=2.0, start_theta=3.14, radius=0.14)

    world.add_robot(robot1)
    world.add_robot(robot2)

    # Initialize the brains (Planners) and give them their targets
    planner1 = GoToGoalPlanner(robot_id=1)
    target1 = (2.0, 2.0) # Drive diagonally across the center

    planner2 = GoToGoalPlanner(robot_id=2)
    target2 = (-2.0, -2.0)

    visualizer = Visualizer(world, bounds=3.0)

    print("Starting simulation...")
    
    while world.time < 20.0:
        commands = {}
        
        # Sense & Plan for Robot 1
        dyn_neighbors1, stat_points1 = world.get_lidar_scan(1)
        commands[1] = planner1.compute_velocity(robot1.state, target1[0], target1[1], dyn_neighbors1, stat_points1)

        # Sense & Plan for Robot 2
        dyn_neighbors2, stat_points2 = world.get_lidar_scan(2)
        commands[2] = planner2.compute_velocity(robot2.state, target2[0], target2[1], dyn_neighbors2, stat_points2)

        # Act
        world.step(commands)
        
        # Draw the world and the goals
        targets = {1: target1, 2: target2}
        visualizer.render(targets=targets)

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()