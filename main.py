import matplotlib.pyplot as plt
from datetime import datetime
from src.simulator.visualizer import Visualizer
from src.utils.logger import MetricsLogger
from src.utils.config_parser import setup_simulation

def main():
    scenario_file = "configs/scenario_02_three_mpc_cross.json"
    world, targets, sim_config = setup_simulation(scenario_file)

    visualizer = Visualizer(world, bounds=3.0)
    logger = MetricsLogger()


    interrupted = False
    try:
        while visualizer.running and world.time < sim_config["max_time"]:
            # Phase 1: Sense and Plan (Simultaneous across all robots)
            for robot in world.robots:
                dyn_neighbors, stat_points = world.get_lidar_scan(robot.id)
                tx, ty = targets[robot.id]
                robot.plan(tx, ty, dyn_neighbors, stat_points)

            # Phase 2: Actuate, record collisions, advance time
            world.step()
            logger.log_step(world, targets)
            visualizer.render(targets=targets)
    except KeyboardInterrupt:
        interrupted = True
        print("Simulation interrupted.")

 

    finished = world.time >= sim_config["max_time"]
    print("Simulation finished." if finished else "Simulation stopped early.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"logs/scenario_02_three_mpc_cross_results_{timestamp}.csv"
    logger.export_to_csv(filename)

    # Keep the final frame open only if the sim ran to completion
    # and the window is still there.
    if finished and not interrupted and visualizer.running:
        plt.ioff()
        plt.show()
    else:
        plt.close("all")

if __name__ == "__main__":
    main()