import csv
import math
import os
from typing import Dict, Tuple
from src.simulator.world import World

class MetricsLogger:
    """
    Observes the simulation and records performance metrics for each robot.
    """
    def __init__(self):
        # Dictionary to store metrics per robot ID
        self.metrics = {}
        # Keep track of where robots were in the previous tick to calculate path length
        self.previous_positions = {}

    def _initialize_robot(self, robot_id: int):
        if robot_id not in self.metrics:
            self.metrics[robot_id] = {
                'status': 'Running',
                'time_to_goal': None,
                'path_length': 0.0,
                'min_safety_margin': float('inf')
            }

    def log_step(self, world: World, targets: Dict[int, Tuple[float, float]], goal_tolerance: float = 0.05):
        """Called every tick to update the running metrics."""
        for robot in world.robots:
            r_id = robot.id
            self._initialize_robot(r_id)

            # Skip updating if the robot has already finished or crashed
            if self.metrics[r_id]['status'] != 'Running':
                continue

            # 1. Check for crashes
            if r_id in world.crashed_robots:
                self.metrics[r_id]['status'] = 'Crashed'
                continue

            # 2. Accumulate Path Length
            curr_pos = (robot.state.x, robot.state.y)
            if r_id in self.previous_positions:
                prev_pos = self.previous_positions[r_id]
                distance_moved = math.hypot(curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1])
                self.metrics[r_id]['path_length'] += distance_moved
            self.previous_positions[r_id] = curr_pos

            # 3. Track Minimum Safety Margin (Distance to closest neighbor)
            for other_robot in world.robots:
                if other_robot.id == r_id:
                    continue
                dist = math.hypot(curr_pos[0] - other_robot.state.x, curr_pos[1] - other_robot.state.y)
                # Subtract radii to get surface-to-surface distance
                surface_dist = dist - (robot.radius + other_robot.radius)
                if surface_dist < self.metrics[r_id]['min_safety_margin']:
                    self.metrics[r_id]['min_safety_margin'] = surface_dist

            # 4. Check if Goal Reached
            target_x, target_y = targets[r_id]
            dist_to_goal = math.hypot(target_x - curr_pos[0], target_y - curr_pos[1])
            if dist_to_goal <= goal_tolerance:
                self.metrics[r_id]['status'] = 'Reached Goal'
                self.metrics[r_id]['time_to_goal'] = world.time

    def export_to_csv(self, filename: str = "simulation_results.csv"):
        """Saves the recorded metrics to a CSV file."""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        headers = ['Robot_ID', 'Status', 'Time_to_Goal_(s)', 'Path_Length_(m)', 'Min_Safety_Margin_(m)']
        
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)
            
            for r_id, data in self.metrics.items():
                time_str = f"{data['time_to_goal']:.2f}" if data['time_to_goal'] else "N/A"
                writer.writerow([
                    r_id,
                    data['status'],
                    time_str,
                    f"{data['path_length']:.2f}",
                    f"{data['min_safety_margin']:.2f}"
                ])
        print(f"\n[LOGGER] Metrics successfully exported to {filename}")