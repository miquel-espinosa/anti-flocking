import matplotlib.path as mpath
import matplotlib.patches as mpatches
import math, sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Circle, Rectangle
from matplotlib.animation import FuncAnimation
import time, getopt

from constants import Constants
from Swarm import *
from functions import *
from rules import *
from plot import *


arguments()


obs1 = Obstacle(ld=[4,4],ru=[18,17])
obs2 = Obstacle(ld=[32,36],ru=[45,48])
obstacles = [obs1, obs2]
# obstacles = []

if Constants.MODE=="continuous": START_TIME = time.monotonic()
if Constants.MODE=="unique": START_TIME = 0

swarm = Swarm(Constants.NUM_UAVS, obstacles)
history_x = [[swarm.pos[i][0]] for i in range(Constants.NUM_UAVS)]
history_y = [[swarm.pos[i][1]] for i in range(Constants.NUM_UAVS)]
history_percentage = [0]


# ======================================================
#              PLOTTING GRAPH AND OBSTACLES
# ======================================================

if Constants.COVERAGE_TEMPERATURE:
    fig_cov_temp, ax_cov_temp, image_cov_temp = plot_coverage_temperature(swarm, START_TIME)

# Drone simulation map
if Constants.TRAJECTORY_PLOT:
    fig_trajectories, ax_trajectories = plot_simulation_map(history_x,history_y)

# Coverage percentage map
if Constants.CUMULATIVE_PERCENTAGE:    
    fig_cov_graph, ax_cov_graph = plt.subplots()
    ax_cov_graph.set_ylim(0,100)   

draw_obstacles(obstacles,ax_trajectories)

agent_colors = assign_agent_colors()

# Iteration counter
iter = 0
FINAL_CONDITION = True

# MAIN EXECUTION LOOP
while FINAL_CONDITION: # 99% coverage

    iter = iter + 1 # Increment iteration counter
    swarm.neighbors = [[] for i in range(Constants.NUM_UAVS)] # Initialization of neighbors

    if iter % 10 == 0: # Print current coverage status
        print("Coverage: ",swarm.coverage_percentage,"%")

    # MAIN LOOP 2
    for agent in range(Constants.NUM_UAVS):

        agent_iteration(START_TIME,swarm,agent)

        # ===================================================
        #                 PLOT GRAPH
        # ===================================================
        history_x[agent].append(swarm.pos[agent][0])
        history_y[agent].append(swarm.pos[agent][1])

        # Trajectory drawing
        if Constants.TRAJECTORY_PLOT:
            ax_trajectories.add_patch(trajectory_patch(history_x,history_y,agent,agent_colors))
            # Add sensor and communication radius circles
            if Constants.ADD_CIRCLES:
                ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_S,edgecolor="cornflowerblue",fill=False,linestyle="--"))
                ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_C,edgecolor="lavender",fill=False,linestyle="--"))

            # Plot goals as scatter points
            # ax_trajectories.scatter(swarm.goal[agent][0],swarm.goal[agent][1], s=1,color=agent_colors[agent], zorder=2)

    # COMPUTE OVERALL COVERAGE PERCENTAGE 
    swarm.coverage_percentage = percentage_covered(swarm)

    # PLOT COVERAGE PERCENTAGE GRAPH
    if Constants.CUMULATIVE_PERCENTAGE:
        history_percentage.append(swarm.coverage_percentage)
        ax_cov_graph.plot(history_percentage, color="b")

    # PLOT AREA COVERAGE TEMPERATURE MAP
    if Constants.COVERAGE_TEMPERATURE:
        # im = ax_cov_temp.imshow(np.rot90(swarm.coverage_map[0]), cmap=plt.cm.RdBu, extent=(-3, 3, 3, -3), interpolation='bilinear')   
        image_cov_temp.set_data(np.rot90(swarm.coverage_map[0]))
        if iter%10==0 and Constants.MODE=="continuous":image_cov_temp.set_clim(Constants.NEG_INF,time.monotonic()-START_TIME)

    # PLOT CURRENT ITERATION AND AGENTS POSITIONS
    if Constants.COVERAGE_TEMPERATURE: fig_cov_temp.canvas.draw_idle()
    if Constants.CUMULATIVE_PERCENTAGE: fig_cov_graph.canvas.draw_idle()
    if Constants.TRAJECTORY_PLOT: fig_trajectories.canvas.draw_idle()
    plt.pause(0.01)

    # FINAL CONDITION:
    if Constants.MODE=="unique": FINAL_CONDITION = (swarm.coverage_percentage < 95)



print()
print(" ============ AREA COVERAGE MISSION COMPLETED ============")
print("       Total time elapsed: ",round(time.monotonic()-START_TIME,2)," seconds" )
print("       Number of total iterations: ", iter)
print("       Final coverage area: ",swarm.coverage_percentage,"%")
print(" =========================================================")
print()
plt.waitforbuttonpress()
time.sleep(5)
