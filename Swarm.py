# -*- coding: utf-8 -*- 
import numpy as np
import random, math

from constants import Constants


class Swarm(object):

    """
        Function to initialize Swarm
            num: number of agents to initialize Swarm
            obstacles: obstacle object to include in coverage initial map
    """
    def __init__(self, num, obstacles):
        
        # φ: Heading angle of UAVS
        self.heading_angle, self.pos = self.init_positions(num)

        # w: Control input for changing heading angle
        self.control_input = np.zeros(num)

        # θ: Difference angle
        self.diff_angle = np.zeros(num)

        # Coverage map
        self.coverage_map = self.init_coverage_map(num, obstacles)
        
        # Actual velocity direction term
        self.vel_actual = np.zeros((num,2))

        # Desired velocity direction term
        self.vel_desired = np.zeros((num,2))
        
        # Obstacle avoidance velocity term
        self.vel_obs = np.zeros((num,2)) 
        
        # Decentering velocity term
        self.vel_dec = np.zeros((num,2))
        
        # Selfishness velocity term
        self.vel_sel = np.zeros((num,2))
        
        # Boundary velocity term
        self.vel_bou = np.zeros((num,2))
        
        # Array for storing neighbors indexes
        self.neighbors = [[] for i in range(num)]
        
        # (x,y) grid goal
        self.goal = np.zeros_like(self.pos)
        
        # (x,y) previous grid goal
        self.prev_goal = np.zeros_like(self.pos)

        # Acumulated percentage coverage
        self.coverage_percentage = 0

        # Instantaneous coverage map
        self.instantaneous_coverage_map = np.zeros((Constants.WIDTH, Constants.LENGTH))

    # Mark with -1 all cells with obstacle inside
    def init_coverage_map(self, num_uavs, obstacles):
        cov_map = np.zeros((Constants.WIDTH,Constants.LENGTH)) 
        for obs in obstacles:
            for row in range(obs.ld[0],obs.ru[0]):
                cov_map[row][obs.ld[1]:obs.ru[1]] = Constants.OBSTACLE_VALUE
        return np.array([cov_map]*num_uavs)

    def init_positions(self, num):
        # return np.random.rand(num,2)*(LENGTH/2)
        # Init all in the middle?
        middle=[Constants.LENGTH/2,Constants.LENGTH/2]
        positions = []
        angles = []

        if Constants.INIT_CIRCUMFERENCE:
            init_angle=(2*math.pi)/num # in radians
            angle=init_angle
            for _ in range(num):
                positions.append((round(middle[0]+math.cos(angle),2),round(middle[1]+math.sin(angle),2)))
                angles.append(math.degrees(angle))
                angle = angle+init_angle

        if Constants.INIT_LINE_UP or Constants.INIT_LINE_DOWN or Constants.INIT_LINE_INTERCHANGED:
            angle=90
            for i in range(num):
                positions.append((middle[0]+i,middle[1]))
                if Constants.INIT_LINE_INTERCHANGED: angle = -angle
                if Constants.INIT_LINE_UP: angle = 90
                if Constants.INIT_LINE_DOWN: angle = -90
                angles.append(angle)
        
        if Constants.INIT_RANDOM:
            for _ in range(num):
                positions.append((Constants.LENGTH/2+random.uniform(-1,1),Constants.LENGTH/2+random.uniform(-1,1)))
                angles.append(360*random.random())

        # if Constants.INIT_CORNER:
        #     x=1
        #     c=Constants.R_S
        #     for i in range(num):
        #         # positions.append((Constants.R_S+random.uniform(-1,1),Constants.R_S+random.uniform(-1,1)))
        #         if x == 1:
        #             positions.append((Constants.R_S,c))
        #         else:
        #             positions.append((c,Constants.R_S))
        #             c=c+Constants.R_S
        #         angles.append(200)
        #         x=-x
        # print(positions)
        # print(angles)

        return np.array(angles), np.array(positions)


