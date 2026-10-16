import matplotlib.pyplot as plt
from src.simulator.visualizer import Visualizer
from src.utils.logger import MetricsLogger
from src.utils.config_parser import setup_simulation

def main():
    scenario_file = "configs/scenario_01_cross.json"
    
    # Let the config parser build everything
    world, targets, planners, sim_config = setup_simulation(scenario_file)

    visualizer = Visualizer(world, bounds=3.0)
    logger = MetricsLogger()

    print(f"Starting simulation loaded from {scenario_file}...")
    
    # Use dynamic max_time from JSON
    while world.time < sim_config["max_time"]:
        commands = {}
        
        for robot in world.robots:
            r_id = robot.id
            dyn_neighbors, stat_points = world.get_lidar_scan(r_id)
            tx, ty = targets[r_id]
            
            # The planner is now dynamically assigned
            commands[r_id] = planners[r_id].compute_velocity(
                robot.state, tx, ty, dyn_neighbors, stat_points
            )

        world.step(commands)
        logger.log_step(world, targets)
        visualizer.render(targets=targets)

    print("Simulation finished.")
    logger.export_to_csv("logs/scenario_01_results.csv")
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()