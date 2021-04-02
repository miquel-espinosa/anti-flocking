import numpy as np
import random

from constants import *
from functions import *


class Swarm(object):

    """
        Function to initialize Swarm
            num: number of agents to initialize Swarm
    """
    def __init__(self, num, obstacles):
        
        # φ: Heading angle of UAVS
        self.heading_angle = np.zeros(num) 
        
        # w: Control input for changing heading angle
        self.control_input = np.zeros(num)

        # θ: Difference angle
        self.diff_angle = np.zeros(num)

        # Coverage map
        self.coverage_map = self.init_coverage_map(num, obstacles)
        
        # Actual velocity direction term
        self.vel_actual = np.ones((num,2))

        # Desired velocity direction term
        self.vel_desired = np.ones((num,2))
        
        # Obstacle avoidance velocity term
        self.vel_obs = np.zeros((num,2)) 
        
        # Decentering velocity term
        self.vel_dec = np.ones((num,2))
        
        # Selfishness velocity term
        self.vel_sel = np.zeros((num,2))
        
        # Boundary velocity term
        self.vel_bou = np.zeros((num,2))
        
        # Array for storing neighbors indexes
        self.neighbors = [[] for i in range(num)]
        
        # Initialize with random positions (not inside any obstacle)
        self.pos = self.init_positions(num, self.coverage_map)
        
        # (x,y) grid goal
        self.goal = np.zeros_like(self.pos)
        
        # (x,y) previous grid goal
        self.prev_goal = np.zeros_like(self.pos)

    # Mark with NEG_INF all cells with obstacle inside
    def init_coverage_map(self, num_uavs, obstacles):
        cov_map = np.zeros((WIDTH,LENGTH)) 
        for obs in obstacles:
            for row in range(obs.ld[0],obs.ru[0]):
                cov_map[row][obs.ld[1]:obs.ru[1]] = NEG_INF
        print(cov_map)
        return np.array([cov_map]*num_uavs)

    # TODO: Initial position of drones 
    #    - Not inside any obstacle
    #    - Not overlapping sensor range ¿?
    # Should they all start in the same point?
    def init_positions(self, num, pos):
        return np.random.rand(num,2)*(LENGTH/2)
        # return np.full((num,2),LENGTH/2) # Init all in the middle?
        # return np.random.rand(num,2)*WIDTH + 1 



class Obstacle(object):

    def __init__(self,ld,ru):
        self.ld = ld # Left-down
        self.ru = ru # Right-up
                

