import numpy as np
from numpy.core.numerictypes import obj2sctype

from constants import *
from functions import *


class Swarm(object):

    """
        Function to initialize Swarm
            num: number of agents to initialize Swarm
    """
    def __init__(self, num, obstacles):
        
        # Heading angle of UAVS φ
        self.heading_angle = np.zeros(num) 
        # Control input for changing heading angle
        self.control_input = np.zeros(num) 
        # Coverage map
        self.coverage_map = self.init_coverage_map(num, obstacles)
        # Desired velocity
        self.vel_desired = np.zeros((num,2))
        # Obstacle avoidance
        self.vel_obs = np.zeros((num,2)) 
        # Decentering
        self.vel_dec = np.zeros((num,2))
        # Selfishness
        self.vel_sel = np.zeros((num,2))
        # Boundary
        self.vel_bou = np.zeros((num,2))
        self.neighbors = [[] for i in range(num)]
        # Initialize with random positions (not inside any obstacle)
        self.pos = self.init_positions(num, self.coverage_map)
        self.goal = self.pos
        self.prev_goal = self.pos

    # Mark with -1 all cells with obstacle inside
    def init_coverage_map(self, num_uavs, obstacles):
        cov_map = np.zeros((WIDTH,LENGTH)) 
        for obs in obstacles:
            for row in range(obs.ld[0],obs.ru[0]):
                cov_map[row][obs.ld[1]:obs.ru[1]] = -1
        return np.array([cov_map]*num_uavs)

    # TODO: Initial position of drones 
    #    - Not inside any obstacle
    #    - Not overlapping sensor range ¿?
    # Should they all start in the same point?
    def init_positions(self, num, pos):
        return np.full((num,2),LENGTH/2) # Init all in the middle?
        # return np.random.rand(num,2)*WIDTH + 1 



class Obstacle(object):

    def __init__(self,ld,ru):
        self.ld = ld # Left-down
        self.ru = ru # Right-up
                

