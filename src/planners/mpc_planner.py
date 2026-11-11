import casadi as ca
import numpy as np
from typing import List
import math

from src.planners.base_planner import BasePlanner
from src.simulator.state import Odometry, Twist
from src.simulator.obstacle import CircularObstacle
from src.prediction.neighbor_predictor import NeighborPredictor

class MpcPlanner(BasePlanner):
    def __init__(self, robot_id: int, dt: float = 0.1):# change later
        super().__init__(robot_id)
        
        self.N = 60           
        self.dt = 0.1                
        
        self.predictor = NeighborPredictor(self.N, self.dt)
        
        self.v_max = 0.6            
        self.v_min = 0       
        self.w_max = 1.2            
        self.w_min = -1.2           
        self.robot_radius = 0.17    
        
        self.d_safe_dyn = (self.robot_radius)*2 + 0.1
        
        self.max_static_obs = 10    
        self.max_dyn_obs = 5        
        
        self.Q_xy = 1.0            
        self.Q_theta = 0.1         
        self.R_v = 0.1              
        self.R_w = 0.1              
        self.S_v = 0.1              
        self.S_w = 0.1              
        
        self.u_prev = np.zeros((2, self.N))
        self._setup_casadi_opti()

    def _setup_casadi_opti(self):
        self.opti = ca.Opti()
        
        self.X = self.opti.variable(3, self.N + 1)  
        self.U = self.opti.variable(2, self.N)      
        
        self.P_start = self.opti.parameter(3)       
        self.P_ref = self.opti.parameter(2)         
        
        self.P_obs_dyn = self.opti.parameter(2 * self.max_dyn_obs, self.N) 
        self.P_obs_static = self.opti.parameter(3, self.max_static_obs) 
        
        self.opti.subject_to(self.X[:, 0] == self.P_start)
        
        cost = 0
        
        for k in range(self.N):
            x_next = self.X[0, k] + self.U[0, k] * ca.cos(self.X[2, k]) * self.dt
            y_next = self.X[1, k] + self.U[0, k] * ca.sin(self.X[2, k]) * self.dt
            theta_next = self.X[2, k] + self.U[1, k] * self.dt
            
            self.opti.subject_to(self.X[0, k+1] == x_next)
            self.opti.subject_to(self.X[1, k+1] == y_next)
            self.opti.subject_to(self.X[2, k+1] == theta_next)
            
            self.opti.subject_to(self.opti.bounded(self.v_min, self.U[0, k], self.v_max))
            self.opti.subject_to(self.opti.bounded(self.w_min, self.U[1, k], self.w_max))
            
            for i in range(self.max_dyn_obs):
                obs_x = self.P_obs_dyn[2 * i, k]      
                obs_y = self.P_obs_dyn[2 * i + 1, k]  
                
                dist_sq_dyn = (self.X[0, k+1] - obs_x)**2 + (self.X[1, k+1] - obs_y)**2
                self.opti.subject_to(dist_sq_dyn >= self.d_safe_dyn**2)
            
            for i in range(self.max_static_obs):
                dist_sq_static = (self.X[0, k+1] - self.P_obs_static[0, i])**2 + (self.X[1, k+1] - self.P_obs_static[1, i])**2
                safe_dist = self.P_obs_static[2, i] + self.robot_radius + 0.15
                self.opti.subject_to(dist_sq_static >= safe_dist**2)
            
            cost += self.Q_xy * ((self.X[0, k+1] - self.P_ref[0])**2 + (self.X[1, k+1] - self.P_ref[1])**2)
            cost += self.R_v * (self.U[0, k]**2) + self.R_w * (self.U[1, k]**2)
            
            if k < self.N - 1:
                cost += self.S_v * ((self.U[0, k+1] - self.U[0, k])**2)
                cost += self.S_w * ((self.U[1, k+1] - self.U[1, k])**2)
                
        self.opti.minimize(cost)
        
        p_opts = {"expand": True}
        s_opts = {"max_iter": 100, "print_level": 0, "acceptable_tol": 1e-3, "acceptable_obj_change_tol": 1e-3}
        self.opti.solver("ipopt", p_opts, s_opts)

    def compute_safe_velocity(self, current_state, pref_vx, pref_vy,
                          neighbors, static_obstacles, goal=None) -> Twist:
        target_x, target_y = goal
        
        
        self.opti.set_value(self.P_start, [current_state.x, current_state.y, current_state.theta])
        self.opti.set_value(self.P_ref, [target_x, target_y])
        
        predicted_neighbors = self.predictor.predict_trajectory(neighbors)        
        dyn_matrix = np.full((2 * self.max_dyn_obs, self.N), 9999.0)
        num_dyn_to_process = min(len(predicted_neighbors), self.max_dyn_obs)
        
        for i in range(num_dyn_to_process):
            dyn_matrix[2*i : 2*i+2, :] = predicted_neighbors[i].expected_path
            
        self.opti.set_value(self.P_obs_dyn, dyn_matrix)
            
        static_matrix = np.full((3, self.max_static_obs), 9999.0)
        num_static_to_process = min(len(static_obstacles), self.max_static_obs)
        
        for i in range(num_static_to_process):
            obs = static_obstacles[i]
            # Handle both CircularObstacle objects and raw tuples/dicts
            if hasattr(obs, 'x'):
                static_matrix[0, i] = obs.x
                static_matrix[1, i] = obs.y
                static_matrix[2, i] = getattr(obs, 'radius', 0.2)
            else:
                # Assumes tuple/list format like (x, y) or (x, y, radius)
                static_matrix[0, i] = obs[0]
                static_matrix[1, i] = obs[1]
                static_matrix[2, i] = obs[2] if len(obs) > 2 else 0.2
            
        self.opti.set_value(self.P_obs_static, static_matrix)
        self.opti.set_initial(self.U, self.u_prev)
        
        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.U)
            
            self.u_prev[:, :-1] = u_opt[:, 1:]
            self.u_prev[:, -1] = u_opt[:, -1]
            
            return Twist(linear_x=float(u_opt[0, 0]), angular_z=float(u_opt[1, 0]))
            
        except Exception as e:
            print(f"[MPC {self.robot_id}] solve failed: {e}")
            print("status:", self.opti.debug.return_status())
            # fa    llback: use the last iterate, or the shifted previous plan
            return Twist(float(self.u_prev[0, 0]), float(self.u_prev[1, 0]))