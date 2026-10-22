import matplotlib.pyplot as plt
from datetime import datetime
from src.simulator.visualizer import Visualizer
from src.utils.logger import MetricsLogger
from src.utils.config_parser import setup_simulation

def main():
    scenario_file = "configs/scenario_01_cross.json"
    world, targets, sim_config = setup_simulation(scenario_file)

    visualizer = Visualizer(world, bounds=2.0)
    logger = MetricsLogger()

    print(f"Starting simulation loaded from {scenario_file}...")
    
    while world.time < sim_config["max_time"]:
        # Phase 1: Sense and Plan (Simultaneous across all robots)
        for robot in world.robots:
            dyn_neighbors, stat_points = world.get_lidar_scan(robot.id)
            tx, ty = targets[robot.id]
            robot.plan(tx, ty, dyn_neighbors, stat_points)

        # Phase 2: Actuate Motors (Simultaneous kinematics update)
        for robot in world.robots:
            robot.step(world.dt)

        world.time += world.dt
        logger.log_step(world, targets)
        visualizer.render(targets=targets)

    print("Simulation finished.")
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"logs/scenario_01_results_{timestamp}.csv"
    logger.export_to_csv(filename)
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()