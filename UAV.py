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
        
        # Initialize with random positions
        self.heading_angle = np.zeros(num) # Heading angle of UAVS
        self.coverage_map = self.init_coverage_map(num, obstacles)
        self.vel_o = np.zeros((num,2))
        self.vel_c = np.zeros((num,2))
        self.vel_s = np.zeros((num,2))
        self.vel_b = np.zeros((num,2))
        self.neighbors = [[] for i in range(num)]
        self.pos = self.init_positions(num, self.coverage_map)

    # Mark with -1 all cells with obstacle inside
    def init_coverage_map(self, num_uavs, obstacles):
        cov_map = np.zeros((WIDTH,LENGTH)) 
        for obs in obstacles:
            for row in range(obs.ld[0],obs.ru[0]):
                cov_map[row][obs.ld[1]:obs.ru[1]] = -1
        return np.array([cov_map]*num_uavs)

    def init_positions(self, num, pos):
        return np.random.rand(num,2)*WIDTH + 1 



class Obstacle(object):

    def __init__(self,ld,ru):
        self.ld = ld # Left-down
        self.ru = ru # Right-up
                

