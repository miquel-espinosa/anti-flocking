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
#              PLOTTING GRAPH AND OBSTACLES
# ======================================================

plt.ion()
fig, ax = plt.subplots()
sc = [[] for i in range(NUM_UAVS)]
for i in range(NUM_UAVS):
    sc[i] = ax.scatter(history_x[i],history_y[i])
    

# plt.clf()
plt.xlim(-10, WIDTH+10)    # set the xlim to xmin, xmax
plt.ylim(-10, LENGTH+10)    # set the xlim to xmin, xmax
plt.gca().set_aspect('equal', adjustable='box')

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


        # =======================================================================
        #           COMPUTE NEIGHBORS + SHARE MAPS + NEIGHBORS AVOIDANCE 
        # =======================================================================

        # Compute neighbors for each uav
        for agent2 in range(agent+1,NUM_UAVS):
            inter_dist = norm(swarm.pos[agent],swarm.pos[agent2])
            if inter_dist<R_C:
                swarm.neighbors[agent].append(agent2)
                swarm.neighbors[agent2].append(agent)
                
                # Sharing of coverage maps
                # TODO: Map sharing should be done after map coverage update ¿?
                new = np.maximum(swarm.coverage_map[agent],swarm.coverage_map[agent2])
                swarm.coverage_map[agent] = new
                swarm.coverage_map[agent2] = new

                # Obstacle avoidance with neighbors:
                # if neighbor is within sensor radius, update obstacle avoidance velocity for both agents
                if inter_dist<R_S:
                    swarm.vel_obs[agent] += s(inter_dist,D_O)*unitary_vector(swarm.pos[agent],swarm.pos[agent2])
                    swarm.vel_obs[agent2] += s(inter_dist,D_O)*unitary_vector(swarm.pos[agent2],swarm.pos[agent])


        # ====================================================================================
        #     OBSTACLES AVOIDANCE + UPDATE COVERAGE MAP (timestamps) + TARGET GRID SELECTION
        # ====================================================================================
        # Compute obstacle avoidance term AND update coverage map AND target grid selection

        # We consider an squared area that is R_S^2
        x_0 = int(np.floor(swarm.pos[agent][0]-2*R_S))
        y_0 = int(np.floor(swarm.pos[agent][1]-2*R_S))
        
        max_fitness = 0 # Variable for storing the max_fitness

        swarm.prev_goal[agent] = swarm.goal[agent] # Save previous goal
        
        for x in range(x_0,4*R_S): 
            for y in range(y_0,4*R_S):

                point = np.array((x,y)) # grid cell under consideration in current iteration
                dist_to_point = norm(swarm.pos[agent],point) # distance agent <--> grid_cell

                # TODO: Should we check if obstacle is inside sensor range? This is given by s(z,d) function
                # if dist_to_point<R_S:

                # ---------- OBSTACLE AVOIDANCE ----------
                # Obstacles are marked with -1 in coverage map
                if swarm.coverage_map[agent][x][y] == -1: 
                    swarm.vel_obs[agent] += s(dist_to_point,D_O) * unitary_vector(swarm.pos[agent],point)
                    
                # ---------- UPDATE COVERAGE MAP ----------
                # If its not an obstacle and is inside the sensor range, update timestamp
                elif dist_to_point < R_S: 
                    swarm.coverage_map[agent][x][y] = time.monotonic()

                # ---------- TARGET GRID SELECTION ----------
                # If not an obstacle and not inside radius, compute heuristics
                # We will perform target grid selection for those cells
                # that are:  R_S < cells < 2*R_S  
                else:
                    # Compute closest neighbor to this point
                    closest = True
                    for neig in swarm.neighbors[agent]:
                        if norm(point,swarm.pos[neig])<dist_to_point:
                            closest = False # If a neighbor is closer than agent (to point x,y)

                    if closest: # Compute fitness value for point
                        # ( - I^p_i)
                        time_diff = time.monotonic()-swarm.coverage_map[agent][x][y] 
                        # distance to previous goal
                        dist_to_prev_goal = norm(np.array([x,y]),swarm.prev_goal[agent])
                        # exponent expression
                        exponent = -ALPHA*dist_to_point-BETA*dist_to_prev_goal
                        # final fitness calculation for point (x,y)
                        fitness = time_diff*(RHO+(1-RHO)*math.exp(exponent))
                        if fitness > max_fitness:
                            max_fitness = fitness
                            swarm.goal[agent] = point # Update goal with point (x,y)
                        
                        # TODO: Avoid oscillation situation --> choose one with better angle
                        # elif fitness == max_fitness:



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
            swarm.vel_dec[agent]=s(norm(mean,swarm.pos[agent]),D_C)*unitary_vector(swarm.pos[agent],mean)


        # ===================================================
        #                 SELFISHNESS TERM
        # ===================================================
        # Compute selfishness term with goal previously computed

        swarm.vel_sel[agent]=unitary_vector(swarm.goal[agent],swarm.pos[agent])


        # ===================================================
        #                 BOUNDARY REPULSTION TERM
        # ===================================================



        
        
        
        # ===================================================
        #         UPDATE FINAL VELOCITY TERM
        # ===================================================

        swarm.vel_desired[agent] = K_O*swarm.vel_obs[agent] + K_C*swarm.vel_dec[agent] + K_S*swarm.vel_sel[agent]
        



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



