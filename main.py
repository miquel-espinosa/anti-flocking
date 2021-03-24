'''
 A Python implementation of
 @article{ganganath2016anti-flocking,
 title={Distributed Anti-Flocking Algorithms for Dynamic Coverage of Mobile Sensor Networks},
 author={Nuwan Ganganath and Chi-Tsun Cheng and Chi K. Tse},
 journal={IEEE Transactions on Industrial Informatics},
 year={2016},
 volume={12},
 number={5},
 pages={1795--1805},
 publisher={IEEE}}

Tested using Python 3.5.2

Original MATLAB implementation can be found at https://github.com/manganganath/distributed_anti-flocking
'''

import numpy as np
import random as rn
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Circle, Rectangle
from func_misc import *

# parameters
num_agents = 5 # number of agents (min = 3)
t_gap = 0.1 #time gap between two iterations (s)
r_c = 20.0 #communication radius
r_s = 5.0 #sensing radius
v_0 = 1.0 #initial velocity
efs = 0.1
c1 = 10.0 #decentering
c2 = 0.5 #actraction to the target
c3 = 0.8 #velocity feedback
rho = 0.2 
sigma1 = 0.04
sigma2 = 0.01
d_a = 1.9*r_s
d_o = 0.9*r_s
map_width = 50.0 #width of a squre map
map_res = 0.5 #width of a grid
neg_inf = -9e+100
arrow_head_length = 1.25  # for representing agents
arrow_head_width = 0.5  # for representing agents

# defining vectors
x = np.zeros((num_agents, 2))        #current position
x_1 = np.zeros((num_agents, 2))      #previous position
v_1 = np.zeros((num_agents, 2))      #previous velocity
tra_dist = np.zeros((num_agents, 1)) #travel distance

cell_map_size = (int(np.floor(map_width/map_res)), int(np.floor(map_width/map_res)), num_agents*2)
cell_map = np.zeros(cell_map_size) #cell_map structure

# initializing the grid centers
map_pos = np.zeros((cell_map_size[0], cell_map_size[1], 2))
for g_i in range(cell_map_size[0]):
    for g_j in range(cell_map_size[1]):
        map_pos[g_i, g_j, 0] = map_res*(g_i-1./2)
        map_pos[g_i, g_j, 1] = map_res*(g_j-1./2)
        
# initializing obstacles (some examples)
obs_map = np.zeros((cell_map_size[0], cell_map_size[1]))
obs_map[64:79, 14:29] = neg_inf
obs_map[21:60, 51:80] = neg_inf

# initializing agents
map_pos_x = map_pos[:, :, 0]
map_pos_y = map_pos[:, :, 1]
map_free_x = map_pos_x[np.where(obs_map != neg_inf)]
map_free_y = map_pos_y[np.where(obs_map != neg_inf)]
free_ind = rn.sample(range(len(map_free_x)), num_agents)
x_1[:,0] = map_free_x[free_ind]
x_1[:,1] = map_free_y[free_ind]

x_r = x_1   #search location
x_r_t = np.zeros((num_agents, 3))    #search location last visited time
[I, J] = np.unravel_index(free_ind,(cell_map_size[0], cell_map_size[1]))
x_r_t[:, 0] = I[0]
x_r_t[:, 1] = J[0]

ini_v_t = 2*np.pi*np.random.uniform(low=0.0, high=1.0, size=num_agents)
ini_v_s = v_0*np.random.uniform(low=0.0, high=1.0, size=num_agents)
v_1[:, 0] = np.multiply(ini_v_s, np.cos(ini_v_t))
v_1[:, 1] = np.multiply(ini_v_s, np.sin(ini_v_t))

# defining agents for obstacles
x_obs = np.asarray([map_pos_x[np.where(obs_map == neg_inf)],map_pos_y[np.where(obs_map == neg_inf)]]).T
num_obs_agents = x_obs.shape[0]
obs_nbr = np.zeros((num_obs_agents,num_agents))

fused_scan_record = np.zeros((cell_map_size[0], cell_map_size[1], 2))
fused_scan_record[:, :, 1] = fused_scan_record[:, :, 1] - obs_map

counter = 1

r_c_sigma = sigma_norm(r_c, efs)
r_s_sigma = sigma_norm(r_s, efs)

# figure
fig, ax = plt.subplots()
plt.xlim(-10, map_width+10)    # set the xlim to xmin, xmax
plt.ylim(-10, map_width+10)    # set the xlim to xmin, xmax
plt.gca().set_aspect('equal', adjustable='box')
# draw obstacles
for o in range(num_obs_agents):
    ax.add_patch(Rectangle((x_obs[o, 0]-map_res/2, x_obs[o, 1]-map_res/2), map_res, map_res, facecolor='none', edgecolor='k'))
plt.xlabel('x (m)')
plt.ylabel('y (m)')
arrow_colors = get_colors(num_agents+1)[1:]

# main loop
while(True):
    u_d = np.zeros((num_agents, 2)) # control input for decentralizing 
    u_o = np.zeros((num_agents, 2)) # control input for obstacle avoidance

    # calculating adjacency matrix 
    dist_gap = get_gap(x_1)
    dist_2 = squd_norm(dist_gap)
    dist = np.sqrt(dist_2)
    adj = action_func(dist, d_a, c1)
    nbr = np.zeros((num_agents, num_agents))
    nbr[dist<=r_c] = 1
    nbr = nbr - np.diag(np.diag(nbr))    # set diagonal to 0
    adj = np.multiply(nbr, adj)
    
    T = counter*t_gap
    
    cell_map = update_individual_record(cell_map, map_pos, x_1, T, r_s)
    
    if np.sum(nbr) > 0:
        cell_map = fuse_record(cell_map, nbr)

    # fusing cell_map for calculations
    fused_scan_record = fuse_all_records(cell_map, num_agents, fused_scan_record)
    
    rem_map = sum(sum(fused_scan_record[:, :, 0]==0))
    cur_covg = sum(sum(fused_scan_record[:, :, 0]==T))
    map_w, map_h =cell_map_size[0], cell_map_size[1]
    cumu_covg = (map_w*map_h-rem_map-num_obs_agents)/((map_w*map_h-num_obs_agents)*0.01)
    inst_covg = cur_covg/((map_w*map_h-num_obs_agents)*0.01)
    
    # selfishness
    u_e = c2*(x_r-x_1)-c3*v_1

    # decentering
    for a in range(num_agents):
        for b in range(num_agents):
            u_d[a, :] = u_d[a, :] + (x_1[b, :]-x_1[a, :])*adj[a, b]/np.sqrt(efs+dist_2[a, b])

    # obstacle avoidance
    if num_obs_agents>0:
        for i in range(num_agents):
            obs_nbr[:, i] = np.sqrt(np.square(x_obs[:, 0]-x_1[i, 0]) + np.square(x_obs[:, 1]-x_1[i, 1]))
            
        obs_nbr[obs_nbr<=d_o] = 1.
        obs_nbr[obs_nbr>d_o] = 0.
        
        for a in range(num_agents):
            obs_nbr_ind = np.where(obs_nbr[:, a]==1.)[0]
            if len(obs_nbr_ind) != 0:
                for b in range(len(obs_nbr_ind)):
                    o_gap = np.zeros((1, 1, 2))
                    o_gap[:, :, 0] = x_obs[obs_nbr_ind[b], 0] - x_1[a, 0]
                    o_gap[:, :, 1] = x_obs[obs_nbr_ind[b], 1] - x_1[a, 1]
                    o_dist_2 = squd_norm(o_gap)
                    u_o[a, :] = u_o[a, :] + action_func(np.sqrt(o_dist_2), d_o, c1)*(x_obs[obs_nbr_ind[b], :]-x_1[a, :])/np.sqrt(efs+o_dist_2)   # obstacle avoidance

                if np.arccos(np.dot(u_o[a, :], v_1[a, :])/(np.linalg.norm(v_1[a, :])*np.linalg.norm(u_o[a, :]))) < np.pi/2.:
                    u_o[a, :] = [0, 0]

    u = u_d + u_o + u_e
    
    # calculating the new position and velocity
    x = x_1 + v_1*t_gap
    v = v_1 + u*t_gap

    v_1 = v
    x_pre = x_1
    x_1 = x
    tra_dist = tra_dist + np.sqrt(np.square(x[:, 0]-x_pre[:, 0]) + np.square(x[:, 1]-x_pre[:, 1])).reshape((num_agents, 1))
    
    # calculate goal distances
    goal_dist = np.zeros((num_agents, num_agents))
    for s in range(num_agents):
        for t in range(num_agents):
            goal_dist[s, t] = np.sqrt((x[s, 0]-x_r[t, 0])**2 + (x[s, 1]-x_r[t, 1])**2)

    for s in range(num_agents):
        obs_map_temp = np.empty_like(obs_map)
        obs_map_temp[:] = obs_map
        temp_cell_map = cell_map[:, :, 2*s]
        recalculate = 0
        if goal_dist[s, s] < r_s:
            recalculate = 1
        elif temp_cell_map[int(x_r_t[s, 0]), int(x_r_t[s, 1])] != x_r_t[s, 2]:
            recalculate = 1
        else:
            for i in range(num_agents):
                if nbr[s,i] == 1:
                    if np.sqrt(np.square(x_r[i, 0]-x_r[s, 0]) + np.square(x_r[i, 1]-x_r[s, 1])) < r_s and goal_dist[i, i] < goal_dist[s, s]:
                        obs_map_temp[np.sqrt(np.square(map_pos[:, :, 0]-x_r[i, 0]) + np.square(map_pos[:, :, 1]-x_r[i, 1])) < r_s] = neg_inf
                        recalculate = 1
                    elif goal_dist[i, s] < goal_dist[i, s] and goal_dist[s, i] < goal_dist[i, i] and recalculate == 0:
                        temp_x_r = x_r[i, :]
                        x_r[i, :] = x_r[s, :]
                        x_r[s, :] = temp_x_r
                        
        if recalculate == 1:
            # calculate distance to each grid 
            tar_dist = np.sqrt(np.square(x[s, 0]-map_pos[:, :, 0]) + np.square(x[s, 1]-map_pos[:, :, 1]))
            # calculate minimum distance to each grid from neighbors
            nbr_tar_dist = np.inf*np.ones((cell_map_size[0], cell_map_size[1]))
            for i in range(num_agents):
                if nbr[s, i] == 1:
                    nbr_tar_dist = np.minimum(nbr_tar_dist, np.sqrt(np.square(x[i, 0]-map_pos[:, :, 0]) + np.square(x[i, 1]-map_pos[:, :, 1])))
            
            obs_map_temp[tar_dist != np.minimum(nbr_tar_dist,tar_dist)] = neg_inf
            pre_tar_dist = np.sqrt(np.square(x_r[s, 0]-map_pos[:, :, 0]) + np.square(x_r[s, 1]-map_pos[:, :, 1]))
            map_d = np.multiply((T-temp_cell_map), (rho+(1-rho)*np.exp(-sigma1*tar_dist-sigma2*pre_tar_dist)))
            map_d1 = np.multiply(map_d, (obs_map_temp+1))
            coor = np.argwhere(map_d1 == np.amax(map_d1))
            ind = rn.sample(range(coor.shape[0]), 1)[0]
            I, J = coor[ind, :]
            x_r[s, :] = map_pos[I, J, :]
            x_r_t[s, :] = [I, J, temp_cell_map[I, J]]
            
    # plotting  
    vdx = []
    vdy = []
    for a in range(num_agents):
        sssp = np.sqrt(np.sum(np.square(v[a, :])))
        if sssp == 0:
            vdx.append(arrow_head_length)
            vdy.append(arrow_head_length)
        else:
            vdx.append(arrow_head_length*v[a, 0]/sssp)
            vdy.append(arrow_head_length*v[a, 1]/sssp)

    gc = []
    for a in range(num_agents):
        gc.append(ax.add_patch(FancyArrow(x[a, 0], x[a, 1], vdx[a], vdy[a], head_width=arrow_head_width, head_length=arrow_head_length, length_includes_head=True,  color=arrow_colors[a])))
        gc.append(ax.add_patch(Circle((x_r[a, 0], x_r[a, 1]), facecolor = 'none',  edgecolor=arrow_colors[a], radius=0.3)))
        for b in range(a):
            if nbr[a, b] == 1.:
                gc.append(ax.add_line(plt.Line2D((x[a, 0], x[b, 0]), (x[a, 1], x[b, 1]), lw=0.5, color='r')))
                
    plt.pause(0.01)
    
    for g in gc:
        g.remove()

    # termination condition
    if cumu_covg==100:
        break
    else:
        counter += 1
