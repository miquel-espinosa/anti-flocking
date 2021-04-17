import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import time
import subprocess

from constants import Constants
from Swarm import Swarm, Obstacle
from rules import arguments, agent_iteration, percentage_covered
from plot import add_video, trajectory_patch, plot_coverage_temperature, plot_simulation_map, draw_obstacles, assign_agent_colors
from functions import cost_fun

arguments()


obs1 = Obstacle(ld=[10,10],ru=[18,17])
obs2 = Obstacle(ld=[32,36],ru=[40,40])
obstacles = [obs1, obs2]
# obstacles = []

if Constants.MODE=="continuous": START_TIME = time.monotonic()
if Constants.MODE=="unique": START_TIME = 0

swarm = Swarm(Constants.NUM_UAVS, obstacles)
history_x = [[swarm.pos[i][0]] for i in range(Constants.NUM_UAVS)]
history_y = [[swarm.pos[i][1]] for i in range(Constants.NUM_UAVS)]
history_percentage = [0]
cost_function = [0]


# ======================================================
#              ANIMATION GRAPHICS
# ======================================================
if Constants.VIDEOS: videos = {'figures': [], 'process': []}


# ======================================================
#              PLOTTING GRAPH AND OBSTACLES
# ======================================================


if Constants.COVERAGE_TEMPERATURE:
    fig_cov_temp, ax_cov_temp, image_cov_temp = plot_coverage_temperature(swarm, START_TIME)
    if Constants.VIDEOS:
        videos['figures'].append(fig_cov_temp)
        videos['process'].append( add_video(*fig_cov_temp.canvas.get_width_height(),"cov_temp") )
    
if Constants.TRAJECTORY_PLOT:
    fig_trajectories, ax_trajectories = plot_simulation_map(history_x,history_y)

if Constants.CUMULATIVE_PERCENTAGE:    
    fig_cov_graph, ax_cov_graph = plt.subplots()
    ax_cov_graph.set_ylim(0,100)   
    ax_cov_graph.set_title("Total Cumulative Area Coverage (%)")
    ax_cov_graph.set_xlabel("Iterations")
    ax_cov_graph.set_ylabel("Area coverage (%)")

if Constants.COST:    
    fig_cost_graph, ax_cost_graph = plt.subplots()

draw_obstacles(obstacles,ax_trajectories)

agent_colors = assign_agent_colors()

# Iteration counter
iter = 0
FINAL_CONDITION = True

# MAIN EXECUTION LOOP
while FINAL_CONDITION: # 99% coverage

    # if iter==20: Constants.NUM_UAVS=Constants.NUM_UAVS-1

    iter = iter + 1 # Increment iteration counter
    swarm.neighbors = [[] for i in range(Constants.NUM_UAVS)] # Initialization of neighbors

    if iter % 10 == 0: # Print current coverage status
        print("Coverage: ",swarm.coverage_percentage,"%")

    # MAIN LOOP 2
    for agent in range(Constants.NUM_UAVS):

        # Most important function
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
            if Constants.CIRCLE_SENSOR:
                ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_S,edgecolor="cornflowerblue",fill=False,linestyle="--"))
            if Constants.CIRCLE_COMMUNICATION:
                ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_C,edgecolor="lavender",fill=False,linestyle="--"))

            # Plot goals as scatter points
            # ax_trajectories.scatter(swarm.goal[agent][0],swarm.goal[agent][1], s=1,color=agent_colors[agent], zorder=2)

    # COMPUTE OVERALL COVERAGE PERCENTAGE 
    swarm.coverage_percentage = percentage_covered(swarm)

    # COMPUTE COST FUNCTION VALUE
    cost_function.append(cost_fun(swarm.coverage_percentage/100,iter))

    # PLOT COVERAGE PERCENTAGE GRAPH
    if Constants.CUMULATIVE_PERCENTAGE:
        history_percentage.append(swarm.coverage_percentage)
        ax_cov_graph.plot(history_percentage, color="b")

    # PLOT COST FUNCTION GRAPH
    if Constants.COST:
        ax_cost_graph.plot(cost_function, color="b")

    # PLOT AREA COVERAGE TEMPERATURE MAP
    if Constants.COVERAGE_TEMPERATURE:
        # im = ax_cov_temp.imshow(np.rot90(swarm.coverage_map[0]), cmap=plt.cm.RdBu, extent=(-3, 3, 3, -3), interpolation='bilinear')   
        image_cov_temp.set_data(np.rot90(swarm.coverage_map[0]))
        if iter%10==0 and Constants.MODE=="continuous":
            image_cov_temp.set_clim(Constants.NEG_INF,time.monotonic()-START_TIME)

    # PLOT CURRENT ITERATION AND AGENTS POSITIONS
    if Constants.COVERAGE_TEMPERATURE: fig_cov_temp.canvas.draw_idle()
    if Constants.CUMULATIVE_PERCENTAGE: fig_cov_graph.canvas.draw_idle()
    if Constants.TRAJECTORY_PLOT: fig_trajectories.canvas.draw_idle()
    plt.pause(0.01)

    
    # extract the image as an ARGB string
    # write to pipe
    if Constants.VIDEOS:
        for index in range(len(videos['figures'])):
            string = videos['figures'][index].canvas.tostring_argb()
            videos['process'][index].stdin.write(string)

    # FINAL CONDITION:
    if Constants.MODE=="unique": FINAL_CONDITION = (swarm.coverage_percentage < 95)

# Videos threads write 
if Constants.VIDEOS:
    for index in range(len(videos['figures'])):
        videos['process'][index].communicate()

print()
print(" ============ AREA COVERAGE MISSION COMPLETED ============")
print("       Total time elapsed: ",round(time.monotonic()-START_TIME,2)," seconds" )
print("       Number of total iterations: ", iter)
print("       Final coverage area: ",swarm.coverage_percentage,"%")
print(" =========================================================")
print()
plt.waitforbuttonpress()
time.sleep(5)
