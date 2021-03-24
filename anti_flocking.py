import numpy as np
import random as rn
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Circle, Rectangle
import matplotlib.path as mpath
import time


from func_misc import *


from constants import *
from UAV import *
from functions import *



obs1 = Obstacle(ld=[140,290],ru=[640,790])
obs2 = Obstacle(ld=[210,600],ru=[510,800])
obstacles = [obs1, obs2]

history_x = [[] for i in range(NUM_UAVS)]
history_y = [[] for i in range(NUM_UAVS)]
swarm = Swarm(NUM_UAVS, obstacles)


# ======================================================
#               PLOTTING GRAPH 
# ======================================================

plt.ion()
fig, ax = plt.subplots()
sc = [[] for i in range(NUM_UAVS)]
for i in range(NUM_UAVS):
    sc[i] = ax.scatter(history_x[i],history_y[i])
    

# plt.clf()
plt.xlim(-10, WIDTH+10)    # set the xlim to xmin, xmax
plt.ylim(-10, LENGTH+10)    # set the xlim to xmin, xmax
# plt.gca().set_aspect('equal', adjustable='box')

for obs in obstacles:
    width = obs.ru[0]-obs.ld[0]
    height = obs.ru[1]-obs.ld[1]
    rect = Rectangle((obs.ld[0], obs.ld[1]), width, height, linewidth=1, edgecolor='r', facecolor='none')
    # Add the patch to the Axes
    ax.add_patch(rect)

plt.draw()



# MAIN EXECUTION LOOP
while True:

    # MAIN LOOP 2
    for agent in range(NUM_UAVS):
        print(swarm.pos[agent])
        # ===================================================
        #                 UPDATE COVERAGE MAP
        # ===================================================

        # Iterate over the radius of agent
        # for x in range(x_0,2*R_S):
        #     for y in range(y_0,2*R_S):
        #         dist = norm(swarm.pos[agent],np.array([x,y]))
        #         if dist<=R_S:
        #             swarm.coverage_map[agent][x][y] = time.monotonic()


        # ===============================================================
        #                 NEIGHBORS AND OBSTACLE AVOIDANCE
        # ===============================================================

        # Compute neighbors for each uav
        for agent2 in range(agent+1,NUM_UAVS):
            inter_dist = norm(swarm.pos[agent],swarm.pos[agent2])
            if inter_dist<R_C:
                swarm.neighbors[agent].append(agent2)
                swarm.neighbors[agent2].append(agent)
                
                # Obstacle avoidance with neighbors:
                # if neighbor is within sensor radius, update obstacle avoidance velocity for both agents
                if inter_dist<R_S:
                    swarm.vel_o[agent] += s(inter_dist,D_O)*unitary_direction(swarm.pos[agent],swarm.pos[agent2])
                    swarm.vel_o[agent2] += s(inter_dist,D_O)*unitary_direction(swarm.pos[agent2],swarm.pos[agent])

        # TODO: Compute obstacle avoidance term 
        # x_0 = int(np.floor(swarm.pos[agent][0]-R_S))
        # y_0 = int(np.floor(swarm.pos[agent][1]-R_S))

        # for obs in obstacles:
        #     if swarm.pos[agent]<

        # for x in range(x_0,2*R_S):
        #     for y in range(y_0,2*R_S):
        #         obs = np.array((x,y))
        #         if norm(swarm.pos[agent],obs)<R_S and obstacle_map[x][y] < 0:
        #             swarm.vel_o[agent] += s(norm(swarm.pos[agent],obs),D_O)*norm(swarm.pos[agent],obs)
        


        # ===============================================================
        #                 DECENTERING TERM
        # ===============================================================

        # Compute Center Of Mass for neighbors
        num_neighbors = len(swarm.neighbors[agent])
        if num_neighbors > 0:
            mean = np.zeros(2)
            for neig in swarm.neighbors[agent]:
                mean[0] += swarm.pos[neig][0]
                mean[1] += swarm.pos[neig][1]
            mean = mean/num_neighbors

            # Update Decentering velocity for current agent
            swarm.vel_c[agent]=s(norm(mean,swarm.pos[agent]),D_C)*unitary_direction(swarm.pos[agent],mean)



       




        # ===================================================
        #                 SELFISHNESS TERM
        # ===================================================



        # ===================================================
        #                 BOUNDARY REPULSTION TERM
        # ===================================================


        
        
        
        # ===================================================
        #         UPDATE POSITIONS WITH NEW VELOCITIES
        # ===================================================

        swarm.pos[agent] = K_O*swarm.vel_o[agent] + K_C*swarm.vel_c[agent]
        print(swarm.pos[agent])



    # ===================================================
    #                 PLOT GRAPH
    # ===================================================
    

    

        # Draw obstacles

        # plt.xlabel('x (m)')
        # plt.ylabel('y (m)')

        history_x[agent].append(swarm.pos[agent][0])
        history_y[agent].append(swarm.pos[agent][1])

        sc[agent].set_offsets(np.c_[history_x[agent],history_y[agent]])




    # PLOT CURRENT ITERATION AND AGENTS POSITIONS 

    fig.canvas.draw_idle()
    plt.pause(0.1)


plt.waitforbuttonpress()



