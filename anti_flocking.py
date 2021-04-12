import matplotlib.path as mpath
import matplotlib.patches as mpatches
import math, sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Circle, Rectangle
import time, colorsys

from constants import *
from Swarm import *
from functions import *

# DEBUGGING
debug = False


obs1 = Obstacle(ld=[4,4],ru=[18,17])
obs2 = Obstacle(ld=[32,36],ru=[45,48])
obstacles = [obs1, obs2]
# obstacles = []

if MODE=="continuous": START_TIME = time.monotonic()
if MODE=="unique": START_TIME = 0

swarm = Swarm(NUM_UAVS, obstacles)
history_x = [[swarm.pos[i][0]] for i in range(NUM_UAVS)]
history_y = [[swarm.pos[i][1]] for i in range(NUM_UAVS)]
history_percentage = [0]
 


# ======================================================
#              PLOTTING GRAPH AND OBSTACLES
# ======================================================

# Coverage temperature map
fig_cov_temp, ax_cov_temp = plt.subplots()
image_cov_temp = ax_cov_temp.imshow(swarm.coverage_map[0], cmap=plt.cm.RdBu, extent=(-3, 3, 3, -3), interpolation='bilinear')
if MODE=="unique": image_cov_temp.set_clim(-1,1)
if MODE=="continuous": image_cov_temp.set_clim(NEG_INF,time.monotonic()-START_TIME)
fig_cov_temp.colorbar(image_cov_temp)


# Drone simulation map
fig_trajectories, ax_trajectories = plt.subplots()
sc = [[] for i in range(NUM_UAVS)]
for i in range(NUM_UAVS):
    sc[i] = ax_trajectories.scatter(history_x[i],history_y[i], s=1)

boundary = Rectangle((0,0), WIDTH, LENGTH, linewidth=1, edgecolor='black', facecolor="lightgrey")
geo_fence = Rectangle((GEO_FENCE_WIDTH,GEO_FENCE_WIDTH), WIDTH-2*GEO_FENCE_WIDTH, LENGTH-2*GEO_FENCE_WIDTH, linewidth=0.5, edgecolor='grey', facecolor="White")
ax_trajectories.add_patch(boundary)
ax_trajectories.add_patch(geo_fence)

ax_trajectories.set_xlim(-10, WIDTH+10)   
ax_trajectories.set_ylim(-10, LENGTH+10)  
ax_trajectories.set_aspect('equal', adjustable='box')

# Coverage percentage map
fig_cov_graph, ax_cov_graph = plt.subplots()
static_percentage_graph = False
ax_cov_graph.set_ylim(0,100)   


for obs in obstacles:
    width = obs.ru[0]-obs.ld[0]
    height = obs.ru[1]-obs.ld[1]
    rect = Rectangle((obs.ld[0], obs.ld[1]), width, height, linewidth=1, edgecolor='black', hatch="////", facecolor="lightgrey")
    # Add the patch to the Axes
    ax_trajectories.add_patch(rect)
    


agent_colors=[]
for c in np.arange(0., 360., 360./NUM_UAVS):
    (r, g, b) = colorsys.hls_to_rgb(c/360., (50 + np.random.rand() * 10)/100., (90 + np.random.rand() * 10)/100.)
    agent_colors.append((r, g, b))

# Iteration counter
iter = 0

# MAIN EXECUTION LOOP
while swarm.coverage_percentage<99: # 99% coverage

    # Increment iteration counter
    iter = iter + 1
    # Initialization of neighbors
    swarm.neighbors = [[] for i in range(NUM_UAVS)]

    # Print current coverage status
    if iter % 10 == 0:
        print("Coverage: ",swarm.coverage_percentage,"%")

    # MAIN LOOP 2
    for agent in range(NUM_UAVS):

        # =======================================================================
        #                               INIT 
        # =======================================================================
        # Save previous goal
        swarm.prev_goal[agent] = swarm.goal[agent]
        # Init obstacle velocity
        swarm.vel_obs[agent] = np.zeros((1,2)) 
        # Set default goal to go to center of area --> only happens when everything is covered
        # swarm.goal[agent] = np.array([LENGTH/2,WIDTH/2])

        # =======================================================================
        #           COMPUTE NEIGHBORS + SHARE MAPS + NEIGHBORS AVOIDANCE 
        # =======================================================================

        # Compute neighbors for each uav
        for agent2 in range(agent+1,NUM_UAVS):
            inter_dist = norm2(swarm.pos[agent],swarm.pos[agent2])
            if inter_dist<R_C:
                swarm.neighbors[agent].append(agent2)
                swarm.neighbors[agent2].append(agent)
                
                # Sharing of coverage maps
                # TODO: Map sharing should be done after map coverage update ¿?
                new = np.maximum(swarm.coverage_map[agent],swarm.coverage_map[agent2])
                swarm.coverage_map[agent] = new
                swarm.coverage_map[agent2] = new

                # if neighbor is within "danger zone", update obstacle avoidance velocity for both agents
                swarm.vel_obs[agent] += s(inter_dist,D_O)*unitary_vector(swarm.pos[agent2],swarm.pos[agent])
                swarm.vel_obs[agent2] += s(inter_dist,D_O)*unitary_vector(swarm.pos[agent],swarm.pos[agent2])


        # ====================================================================================
        #     OBSTACLES AVOIDANCE + UPDATE COVERAGE MAP (timestamps) + TARGET GRID SELECTION
        # ====================================================================================
        # Compute obstacle avoidance term AND update coverage map AND target grid selection

        # ******************* TODO: TO REMOVE *************************************
        # BOUDING AREA: FOR OBSTACLE SCANNING, COVERAGE UPDATING AND SELECTING CELL GOAL
        # We consider an squared area that is R_S^2
        # Loop over agent square-area surroundings
        # for x in range(x0_bounded,x_upper): 
        #     for y in range(y0_bounded,y_upper):
        # x0 = int(np.floor(swarm.pos[agent][0]-2*R_S))
        # y0 = int(np.floor(swarm.pos[agent][1]-2*R_S))
        # x0_bounded = max(x0,0)
        # y0_bounded = max(y0,0)
        # x_upper = min(x0+4*R_S,WIDTH) 
        # y_upper = min(y0+4*R_S,LENGTH)
        # ************************************************************************
        
        # AUX LOCAL VARIABLES
        max_fitness = 0 # Variable for storing the max_fitness
        best_angle = 180 # Variable for storing the best angle for choosing optimum goal cell
        
        # Loop over entire coverage area
        for x in range(0,WIDTH): 
            for y in range(0,LENGTH):

                point = np.array((x,y)) # grid cell under consideration in current iteration
                dist_to_point = norm2(swarm.pos[agent],point) # distance agent <--> grid_cell

                # ---------- OBSTACLE AVOIDANCE ----------
                # Obstacles are marked with NEG_INF in coverage map
                # If obstacles are inside D_0 radius, repulsion
                if swarm.coverage_map[agent][x][y] == NEG_INF: 
                    swarm.vel_obs[agent] += (s(dist_to_point,D_O) * unitary_vector(point,swarm.pos[agent]))
                    
                # ---------- UPDATE COVERAGE MAP ----------
                # If inside not an obstacle and is inside the sensor range, update timestamp
                elif dist_to_point < R_S: 
                    if MODE=="continuous": swarm.coverage_map[agent][x][y] = time.monotonic()-START_TIME
                    if MODE=="unique": swarm.coverage_map[agent][x][y] = 1
                    # image_cov_temp.set_data((x,y))
                    
                # ---------- TARGET GRID SELECTION ----------
                # If not an obstacle and not inside radius, compute heuristics
                # We will perform target grid selection for those cells
                # that are:  R_S < cells < 2*R_S  
                elif (swarm.coverage_map[agent][x][y] != NEG_INF) and (not outside_area(x,y)):
                    # Compute closest neighbor to this point
                    closest = True
                    for neig in swarm.neighbors[agent]:
                        if norm2(point,swarm.pos[neig])<dist_to_point:
                            closest = False # If a neighbor is closer than agent (to point x,y)

                    if closest: # Compute fitness value for point
                        if MODE=="continuous":
                            # ( - I^p_i)
                            time_diff = time.monotonic()-START_TIME-swarm.coverage_map[agent][x][y] 
                            # time_diff = 1-swarm.coverage_map[agent][x][y] 
                            # distance to previous goal
                            dist_to_prev_goal = norm2(np.array([x,y]),swarm.prev_goal[agent])
                            # exponent expression
                            exponent = -ALPHA*dist_to_point-BETA*dist_to_prev_goal
                            # final fitness calculation for point (x,y)
                            fitness = time_diff*(RHO+(1-RHO)*math.exp(exponent))
                        
                        """ Value 1: Not covered
                            Value 0: Already covered"""
                        if MODE=="unique":
                            fitness = 1-swarm.coverage_map[agent][x][y]

                        if fitness > max_fitness:
                            max_fitness = fitness
                            swarm.goal[agent] = point # Update goal with point (x,y)
                            # Compute unitary velocity vector to goal
                            vel_goal=unitary_vector(swarm.pos[agent],point)
                            # Update best current angle to point to best current goal
                            best_angle = angle_between(swarm.vel_actual[agent],vel_goal)
                        
                        # TODO: It needs to be adjusted correctly
                        # In order to avoid oscillation situation: choose the goal with better angle
                        # We will check with a rounding of two decimal places for the fitnesses values

                        # elif (fitness == max_fitness) and (fitness != 0):
                        # elif fitness == max_fitness:
                        elif abs(max_fitness-fitness)<0.005: # TODO: how to know this value
                            # print("HEHEHEHEHE",iter)
                            # print(fitness-max_fitness)
                            # Compute unitary velocity vector to possible goal
                            vel_goal=unitary_vector(swarm.pos[agent],point)
                            # Compute angle between actual velocity and possible goal
                            angle = angle_between(vel_goal,swarm.vel_actual[agent])

                            # Compare angle with best_current_angle
                            if angle < best_angle:
                                best_angle = angle
                                max_fitness = fitness
                                swarm.goal[agent] = point # Update goal with point (x,y)
                        
                        

        # ===============================================================
        #                 DECENTERING TERM
        # ===============================================================

        # Compute Center Of Mass for neighbors
        num_neighbors = len(swarm.neighbors[agent])
        if num_neighbors > 0:
            mean = np.array(swarm.pos[agent])
            for neig in swarm.neighbors[agent]:
                mean[0] += swarm.pos[neig][0]
                mean[1] += swarm.pos[neig][1]
            mean = mean/(num_neighbors+1)

            # Update Decentering velocity for current agent
            swarm.vel_dec[agent]=s(norm2(mean,swarm.pos[agent]),D_C)*unitary_vector(mean,swarm.pos[agent])


        # ===================================================
        #                 SELFISHNESS TERM
        # ===================================================
        # Compute selfishness term with goal previously computed

        swarm.vel_sel[agent]=unitary_vector(swarm.pos[agent],swarm.goal[agent])


        # ===================================================
        #                 BOUNDARY REPULSTION TERM
        # ===================================================

        if outside_area(*swarm.pos[agent]):
            p_m = np.array((WIDTH/2,LENGTH/2))
            swarm.vel_bou[agent] = Q_M*unitary_vector(swarm.pos[agent],p_m) 

        
        
        
        # ===================================================
        #         FINAL VELOCITY DIRECTION TERM
        # ===================================================

        swarm.vel_desired[agent] = K_O*swarm.vel_obs[agent] \
                                + K_C*swarm.vel_dec[agent] \
                                + K_S*swarm.vel_sel[agent] \
                                + K_B*swarm.vel_bou[agent]
        
        

        # ===================================================
        #             COMPUTE DIFFERENCE ANGLE 
        # ===================================================

        # Actual velocity
        swarm.vel_actual[agent] = np.array([math.cos(math.radians(swarm.heading_angle[agent])),math.sin(math.radians(swarm.heading_angle[agent]))])

        if debug: print("VEL DESIRED: ",swarm.vel_desired[agent])
        if debug: print("VEL ACTUAL: ",swarm.vel_actual[agent])
        

        # Compute angle between actual velocity and desired velocity terms
        # Positive or negative sign
        sign_result = math.copysign(1,np.cross(np.append(swarm.vel_actual[agent],0),np.append(swarm.vel_desired[agent],0))[2])
        # Final computation for agent angle
        swarm.diff_angle[agent] = angle_between(swarm.vel_desired[agent],swarm.vel_actual[agent])*sign_result
        if debug: print("DIFF ANGLE:",swarm.diff_angle[agent])
        


        # ===================================================
        #                  CONTROL INPUT
        # ===================================================

        if swarm.diff_angle[agent]>=0:
            swarm.control_input[agent] = min(W_MAX,K_W*swarm.diff_angle[agent])
        else:
            swarm.control_input[agent] = max(-W_MAX,K_W*swarm.diff_angle[agent])

        if debug: print("CONTROL INPUT: ",swarm.control_input[agent])

        # ===================================================
        #         KINEMATIC LAWS - UPDATE POSITION
        # ===================================================

        # IMPORTANT!!! --> First we must update the heading angle
        # Heading angle
        swarm.heading_angle[agent] = swarm.heading_angle[agent] + CONSTANT_VELOCITY*TIME_STEP*swarm.control_input[agent]
        if debug: print("NEXT HEADING ANGLE: ",swarm.heading_angle[agent])

        # With the new updated heading angle, we compute the next agent position
        # X Position
        swarm.pos[agent][0] = swarm.pos[agent][0] + CONSTANT_VELOCITY*TIME_STEP*math.cos(math.radians(swarm.heading_angle[agent]))
        # Y Position
        swarm.pos[agent][1] = swarm.pos[agent][1] + CONSTANT_VELOCITY*TIME_STEP*math.sin(math.radians(swarm.heading_angle[agent]))



    # ===================================================
    #                 PLOT GRAPH
    # ===================================================
    

        # Draw obstacles

        # plt.xlabel('x (m)')
        # plt.ylabel('y (m)')

        history_x[agent].append(swarm.pos[agent][0])
        history_y[agent].append(swarm.pos[agent][1])

        # sc[agent].set_offsets(np.c_[history_x[agent],history_y[agent]])
        # prueba = ax.scatter(swarm.pos[agent][0],swarm.pos[agent][1], s=1, color=agent_colors[agent])
        string_path_data = [
            (mpath.Path.MOVETO, (history_x[agent][-2],history_y[agent][-2])),
            (mpath.Path.CURVE3, (history_x[agent][-2],history_y[agent][-2])),
            (mpath.Path.CURVE3, (history_x[agent][-1],history_y[agent][-1]))]

        codes, verts = zip(*string_path_data)
        string_path = mpath.Path(verts, codes)
        patch = mpatches.PathPatch(string_path, color=agent_colors[agent], lw=2)

        ax_trajectories.add_patch(patch)
        # Add circles
        # ax_trajectories.add_patch(Circle(swarm.pos[agent],R_S,edgecolor="cornflowerblue",fill=False,linestyle="--"))
        # ax_trajectories.add_patch(Circle(swarm.pos[agent],R_C,edgecolor="lavender",fill=False,linestyle="--"))


        if debug: print("------------------------")

        # ax_trajectories.scatter(swarm.goal[agent][0],swarm.goal[agent][1], s=1,color=agent_colors[agent], zorder=2)
        
        # ============================ END OF AGENT ITERATION ===============================


    if debug: print("POS AGENT 1: ",swarm.pos[0])
    if debug: print("COVERED: ",swarm.coverage_map[0][int(swarm.goal[0][0])][int(swarm.goal[0][1])])

    # desired_vel = ax_trajectories.scatter(swarm.pos[0][0]+swarm.vel_actual[0][0],swarm.pos[0][1]+swarm.vel_actual[0][1], s=1, zorder=2)
    

    # COMPUTE OVERALL COVERAGE PERCENTAGE 
    area = WIDTH * LENGTH
    if NUM_UAVS >= 2:
        aux_max = np.maximum(swarm.coverage_map[0],swarm.coverage_map[1])
        for i in range(2,NUM_UAVS): aux_max = np.maximum(aux_max,swarm.coverage_map[i])
    else:
        aux_max = swarm.coverage_map[0]
    swarm.coverage_percentage = (np.count_nonzero(aux_max)/area)*100 
    history_percentage.append(swarm.coverage_percentage)
    if debug: print("COVERAGE: ",swarm.coverage_percentage,"%")

    # PLOT COVERAGE PERCENTAGE GRAPH
    if static_percentage_graph:
        string_percentage = [
            (mpath.Path.MOVETO, (iter-1,history_percentage[-2])),
            (mpath.Path.CURVE3, (iter-1,history_percentage[-2])),
            (mpath.Path.CURVE3, (iter,history_percentage[-1]))]

        cod, ver = zip(*string_percentage)
        line = mpath.Path(ver, cod)
        patch_line = mpatches.PathPatch(line, color="b", lw=2)
        length = len(history_percentage)
        ax_cov_graph.set_xlim(max(0,length-200),max(length,200))
        ax_cov_graph.add_patch(patch_line)
    else:
        ax_cov_graph.plot(history_percentage, color="b")

    # PLOT AREA COVERAGE TEMPERATURE MAP
    # im = ax_cov_temp.imshow(np.rot90(swarm.coverage_map[0]), cmap=plt.cm.RdBu, extent=(-3, 3, 3, -3), interpolation='bilinear')
    image_cov_temp.set_data(np.rot90(swarm.coverage_map[0])) 
    if iter%10==0 and MODE=="continuous":image_cov_temp.set_clim(NEG_INF,time.monotonic()-START_TIME)
    # plt.draw()
    # print(swarm.goal)

    # PLOT CURRENT ITERATION AND AGENTS POSITIONS
    fig_cov_temp.canvas.draw_idle()
    fig_cov_graph.canvas.draw_idle()
    fig_trajectories.canvas.draw_idle()
    plt.pause(0.01)

    

print()
print(" ============ AREA COVERAGE MISSION COMPLETED ============")
print("       Total time elapsed: ",round(time.monotonic()-START_TIME,2)," seconds" )
print("       Number of total iterations: ", iter)
print("       Final coverage area: ",swarm.coverage_percentage,"%")
print(" =========================================================")
print()
plt.waitforbuttonpress()
time.sleep(5)
