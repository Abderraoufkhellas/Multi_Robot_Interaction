from src.simulator.world import World
from src.simulator.robot import DifferentialRobot
from src.simulator.visualizer import Visualizer
from src.simulator.state import Twist
import matplotlib.pyplot as plt

def main():
    # Initialize the simulation environment (10 Hz clock)
    world = World(dt=0.01)

    # Spawn two robots 
    robot1 = DifferentialRobot(robot_id=1, start_x=-2.0, start_y=1.0, start_theta=0.0, radius=0.14)
    robot2 = DifferentialRobot(robot_id=2, start_x=0.0, start_y=-2.0, start_theta=1.57, radius=0.14)

    world.add_robot(robot1)
    world.add_robot(robot2)

    #  Initialize the visualizer
    visualizer = Visualizer(world, bounds=4.0)

    print("Starting simulation...")
    
    #The Master Execution Loop (for 15  seconds)
    while world.time < 15.0:
        
        # Dummy Planners: Hardcode commands to see them move
        # Robot 1 will drive in a continuous circle
        cmd_r1 = Twist(linear_x=0.5, angular_z=0.4)
        
        # Robot 2 will drive straight, then brake at 7.5 seconds
        if world.time < 7.5:
            cmd_r2 = Twist(linear_x=0.8, angular_z=0.0)
        else:
            cmd_r2 = Twist(linear_x=0.0, angular_z=0.0)

        # B. Bundle commands into the dictionary World expects
        commands = {
            1: cmd_r1,
            2: cmd_r2
        }

        # Advance physics by 1 tick
        world.step(commands)
        
        # D. Redraw the screen
        visualizer.render()

    print("Simulation finished.")
    
    # 5. Keep the final plot open so it doesn't instantly close when done
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()