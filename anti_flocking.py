import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import time

from constants import Constants
from Swarm import Swarm
from Obstacle import Obstacle
from rules import arguments, agent_iteration, percentage_covered
from plot import add_video, trajectory_patch, plot_coverage_temperature, plot_simulation_map, draw_obstacles, assign_agent_colors, get_screen_dimensions

# Command line arguments processing
# Files and directories creation
arguments()

# ======================================================
#           SWARM AND OBSTACLES INITIALIZATION
# ======================================================

obs1 = Obstacle(ld=[10,10],ru=[18,17])
obs2 = Obstacle(ld=[32,36],ru=[40,40])

obs3 = Obstacle(ld=[37,6],ru=[43,43])
obs4 = Obstacle(ld=[25,30],ru=[35,40])
obs5 = Obstacle(ld=[7,20],ru=[15,40])

o11 = Obstacle(ld=[10,40],ru=[12,42])
o12 = Obstacle(ld=[20,40],ru=[22,42])
o13 = Obstacle(ld=[30,40],ru=[32,42])
o14 = Obstacle(ld=[40,40],ru=[42,42])

o21 = Obstacle(ld=[12,30],ru=[14,32])
o22 = Obstacle(ld=[22,30],ru=[24,32])
o23 = Obstacle(ld=[32,30],ru=[34,32])
o24 = Obstacle(ld=[42,30],ru=[44,32])

o31 = Obstacle(ld=[10,20],ru=[12,22])
o32 = Obstacle(ld=[20,20],ru=[22,22])
o33 = Obstacle(ld=[30,20],ru=[32,22])
o34 = Obstacle(ld=[40,20],ru=[42,22])

o41 = Obstacle(ld=[12,10],ru=[14,12])
o42 = Obstacle(ld=[22,10],ru=[24,12])
o43 = Obstacle(ld=[32,10],ru=[34,12])
o44 = Obstacle(ld=[42,10],ru=[44,12])

# obstacles = [obs1, obs2]
# obstacles = [obs3, obs4, obs5]
obstacles = [o11,o12,o13,o14,
             o21,o22,o23,o24,
             o31,o32,o33,o34,
             o41,o42,o43,o44]
# obstacles = []

if Constants.MODE=="continuous": START_TIME = time.monotonic()
if Constants.MODE=="unique": START_TIME = 0

swarm = Swarm(Constants.NUM_UAVS, obstacles)
history_x = [[swarm.pos[i][0]] for i in range(Constants.NUM_UAVS)]
history_y = [[swarm.pos[i][1]] for i in range(Constants.NUM_UAVS)]
history_percentage = [0]
instant_coverage = [0]


# ======================================================
#              PLOTTING GRAPH AND OBSTACLES
# ======================================================

fig, ((ax_cov_temp, ax_trajectories), (ax_inst_graph, ax_cov_graph)) = plt.subplots(2, 2, gridspec_kw={'height_ratios': [3, 1]})
fig.tight_layout()

width_in, height_in = get_screen_dimensions()
fig.set_size_inches(round(width_in)-1,round(height_in)-1)

# Set graphics window size fixed when recording video to avoid crashing
if Constants.VIDEO:
    win = fig.canvas.window()
    win.setFixedSize(win.size())

width, height = fig.canvas.get_width_height()
if Constants.VIDEO: video = add_video(width,height,str(Constants.RESULTS_DIR+"/"+Constants.FILE_NAME))

# Create and init plots
if Constants.COVERAGE_TEMPERATURE:
    ax_cov_temp, image_cov_temp = plot_coverage_temperature(fig, ax_cov_temp, swarm, START_TIME)
    
if Constants.TRAJECTORY_PLOT:
    ax_trajectories = plot_simulation_map(ax_trajectories, history_x,history_y)

if Constants.CUMULATIVE_PERCENTAGE:
    ax_cov_graph.set_ylim(0,100)   
    ax_cov_graph.yaxis.tick_right()
    ax_cov_graph.set_title("Total Cumulative Area Coverage (%)")
    ax_cov_graph.set_xlabel("Iterations")
    ax_cov_graph.set_ylabel("Area coverage (%)")
    
if Constants.INSTANTANEOUS_PERCENTAGE:
    # ax_inst_graph.set_ylim(0,100)   
    ax_inst_graph.set_title("Instantaneous Area Coverage (%) for whole UAV Swarm")
    ax_inst_graph.set_xlabel("Iterations")
    ax_inst_graph.set_ylabel("Instantaneous Area Coverage")


draw_obstacles(obstacles,ax_trajectories)
agent_colors = assign_agent_colors()

# Iteration counter
iter = 0
FINAL_CONDITION = True

# MAIN EXECUTION LOOP
while FINAL_CONDITION: # 95% coverage or 400 max iterations = 

    if Constants.SIMULATE_FAILURES and Constants.NUM_UAVS > 1 and iter > 1:
        # Kill UAV every 50 iterations
        if iter==50:
            # --------- Failures ------------
            Constants.NUM_UAVS = Constants.NUM_UAVS-1
            ax_trajectories.scatter(*swarm.pos[Constants.NUM_UAVS],color='r',marker="x", zorder=2, s=130, linewidth=3)
        if iter==100:
            # --------- New members ------------
            Constants.NUM_UAVS = Constants.NUM_UAVS+1
            ax_trajectories.scatter(*swarm.pos[Constants.NUM_UAVS-1],color='black',marker="o", zorder=2, s=60)

    iter = iter + 1 # Increment iteration counter
    swarm.neighbors = [[] for i in range(Constants.NUM_UAVS)] # Initialization of neighbors
    swarm.instantaneous_coverage_map = np.zeros((Constants.WIDTH, Constants.LENGTH)) # Initialize instantaneous coverage map

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
            if Constants.CIRCLE_COMMUNICATION and Constants.ALWAYS_COMMUNICATION==False:
                ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_C,edgecolor="lavender",fill=False,linestyle="--"))

            # Plot goals as scatter points
            # ax_trajectories.scatter(swarm.goal[agent][0],swarm.goal[agent][1], s=1,color=agent_colors[agent], zorder=2)

    # COMPUTE OVERALL COVERAGE PERCENTAGE 
    total_coverage_map, swarm.coverage_percentage = percentage_covered(swarm)

    # COMPUTE INSTANTANEOUS COVERAGE
    instant_coverage_percentage = (np.count_nonzero(swarm.instantaneous_coverage_map==1)/(Constants.WIDTH*Constants.LENGTH))*100
    instant_coverage.append(instant_coverage_percentage)

    # PLOT COVERAGE PERCENTAGE GRAPH
    if Constants.CUMULATIVE_PERCENTAGE:
        history_percentage.append(swarm.coverage_percentage)
        if Constants.PLOTS:
            ax_cov_graph.plot(history_percentage, color="b")

            # CURRENT COVERAGE PERCENTAGE
            ax_cov_graph.annotate(" "+str(round(swarm.coverage_percentage,2))+"% ",
                xy=(0.86,0.9), xycoords='axes fraction',
                size=14,
                bbox=dict(boxstyle="round", fc=(0.5, 0.8, 1.0), ec="none"))

    # PLOT INSTANTANEOUS COVERAGE (%)
    if Constants.INSTANTANEOUS_PERCENTAGE and Constants.PLOTS:
        ax_inst_graph.plot(instant_coverage, color="b")

        ax_inst_graph.annotate(" "+str(round(instant_coverage_percentage,2))+"% ",
            xy=(0.86,0.1), xycoords='axes fraction',
            size=14,
            bbox=dict(boxstyle="round", fc=(0.5, 0.8, 1.0), ec="none"))

    # PLOT AREA COVERAGE TEMPERATURE MAP
    if Constants.COVERAGE_TEMPERATURE and Constants.PLOTS:
        image_cov_temp.set_data(np.rot90(total_coverage_map))
        # if iter%10==0 and Constants.MODE=="continuous":
        if Constants.MODE=="continuous":
            image_cov_temp.set_clim(Constants.OBSTACLE_VALUE,time.monotonic()-START_TIME)

    # Update canvas with new changes
    if Constants.PLOTS:
        fig.canvas.draw_idle()
        plt.pause(0.01)

    if Constants.VIDEO and Constants.PLOTS:
        string = fig.canvas.tostring_argb() # Extract the image as an ARGB string
        video.stdin.write(string) # Write to pipe

    # FINAL CONDITION to exit loop
    if Constants.MAX_ITERATIONS <= iter or Constants.MAX_COVERAGE <= swarm.coverage_percentage: 
        FINAL_CONDITION = False


# Draw canvas for final picture when real-time plots turned off
if not Constants.PLOTS:
    ax_cov_graph.plot(history_percentage, color="b")
    ax_cov_graph.annotate(" "+str(round(swarm.coverage_percentage,2))+"% ", xy=(0.86,0.9), xycoords='axes fraction', size=14, bbox=dict(boxstyle="round", fc=(0.5, 0.8, 1.0), ec="none"))
    ax_inst_graph.plot(instant_coverage, color="b")
    ax_inst_graph.annotate(" "+str(round(instant_coverage_percentage,2))+"% ", xy=(0.86,0.1), xycoords='axes fraction', size=14, bbox=dict(boxstyle="round", fc=(0.5, 0.8, 1.0), ec="none"))
    image_cov_temp.set_data(np.rot90(total_coverage_map))
    if Constants.MODE=="continuous":
        image_cov_temp.set_clim(Constants.OBSTACLE_VALUE,time.monotonic()-START_TIME)
    ax_trajectories.add_patch(Circle(swarm.pos[agent],Constants.R_S,edgecolor="cornflowerblue",fill=False,linestyle="--"))

fig.savefig(str(Constants.RESULTS_DIR+"/"+Constants.FILE_NAME), bbox_inches="tight")

# Video thread write 
if Constants.VIDEO:
    video.communicate()


exec_time = str(round(time.monotonic()-START_TIME,2))
total_iter = str(iter)
total_cov_area = str(round(swarm.coverage_percentage,2))
average_inst_cov_area = str(round(sum(instant_coverage)/len(instant_coverage),2))
mission_time = str(iter*Constants.TIME_STEP)

file = open(str(Constants.RESULTS_DIR+"/"+Constants.FILE_NAME), "w")
file.write("\n")
file.write(" ============ AREA COVERAGE MISSION COMPLETED ============ \n")
file.write(str("       (Exec time: "+exec_time+" s)\n"))
file.write(str("       Number of UAVs: "+str(Constants.NUM_UAVS)+"\n"))
file.write(str("       Coverage Mode: "+str(Constants.MODE)+"\n"))
file.write(str("       Always communication: "+str(Constants.ALWAYS_COMMUNICATION)+"\n"))
file.write(str("       Number of total iterations: " + total_iter +"\n"))
file.write(str("       Final coverage area: " + total_cov_area +"%\n"))
file.write(str("       Average instantaneous coverage area: " + average_inst_cov_area +"%\n"))
file.write(str("       Mission time: " + mission_time +" s\n"))
file.write(" =========================================================\n")
file.write("\n")
file.close()
time.sleep(1)


print(f"{exec_time},{str(Constants.NUM_UAVS)},{str(Constants.MODE)},{str(Constants.ALWAYS_COMMUNICATION)},{total_iter},{total_cov_area},{average_inst_cov_area},{mission_time},{Constants.MAX_COVERAGE}")