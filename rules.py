import time, getopt, sys

from functions import * 


def init(swarm,agent):
    """ Init params function """
    # Save previous goal
    swarm.prev_goal[agent] = swarm.goal[agent]
    # Init obstacle velocity
    swarm.vel_obs[agent] = np.zeros((1,2)) 


def get_neighbors(agent,swarm):
    """ 
        + COMPUTE NEIGHBORS
        + SHARE MAPS
        + NEIGHBORS AVOIDANCE 
    """
    # Compute neighbors for each uav
    for agent2 in range(agent+1,Constants.NUM_UAVS):
        inter_dist = norm2(swarm.pos[agent],swarm.pos[agent2])
        if inter_dist<Constants.R_C:
            swarm.neighbors[agent].append(agent2)
            swarm.neighbors[agent2].append(agent)

            # if neighbor is within "danger zone", update obstacle avoidance velocity for both agents
            swarm.vel_obs[agent] += s(inter_dist,Constants.D_O)*unitary_vector(swarm.pos[agent2],swarm.pos[agent])
            swarm.vel_obs[agent2] += s(inter_dist,Constants.D_O)*unitary_vector(swarm.pos[agent],swarm.pos[agent2])


def compute_fitness(swarm,agent,START_TIME,point,x,y,dist_to_point):
    if Constants.MODE=="continuous": # surveillance mode (with time)
        # ( - I^p_i)
        # This will ensure that the agent prioritizes cells that have not been covered yet
        if swarm.coverage_map[agent][x][y] == 0 and Constants.GOAL_OPTIMIZATION:
            time_diff = (time.monotonic()-START_TIME)*100
            if Constants.GOAL_OPTIMIZATION:
                # Indicates the number of cells withing R_S radius of selected point that have already been covered in the past
                adjacent_coverage=radius_covered(swarm.coverage_map[agent],point)
                # Penalize points with neighbors already covered
                time_diff = time_diff - adjacent_coverage
        else:
            time_diff = time.monotonic()-START_TIME-swarm.coverage_map[agent][x][y] 
        # distance to previous goal
        dist_to_prev_goal = norm2(np.array([x,y]),swarm.prev_goal[agent])
        # exponent expression
        exponent = -Constants.ALPHA*dist_to_point-Constants.BETA*dist_to_prev_goal
        # final fitness calculation for point (x,y)
        fitness = time_diff*(Constants.RHO+(1-Constants.RHO)*math.exp(exponent))
    
    """ Value 1: Not covered
        Value 0: Already covered"""
    if Constants.MODE=="unique": # no time consideration
        fitness = 1-swarm.coverage_map[agent][x][y]
        if Constants.GOAL_OPTIMIZATION:
            # Indicates the number of cells withing R_S radius of selected point that have already been covered in the past
            adjacent_coverage=radius_covered(swarm.coverage_map[agent],point)
            # To avoid division by zero
            if adjacent_coverage==0: adjacent_coverage = 1
            # Penalize points with neighbors already covered
            fitness = fitness + (1/adjacent_coverage)*10

    return fitness

def decentering_velocity(swarm, agent):
    num_neighbors = len(swarm.neighbors[agent])
    if num_neighbors > 0:
        mean = np.array(swarm.pos[agent])
        for neig in swarm.neighbors[agent]:
            mean[0] += swarm.pos[neig][0]
            mean[1] += swarm.pos[neig][1]

            # Sharing of coverage maps between neighbors
            new = np.maximum(swarm.coverage_map[agent],swarm.coverage_map[neig])
            swarm.coverage_map[agent] = new
            swarm.coverage_map[neig] = new

        mean = mean/(num_neighbors+1)

        # Update Decentering velocity for current agent
        swarm.vel_dec[agent]=s(norm2(mean,swarm.pos[agent]),Constants.D_C)*unitary_vector(mean,swarm.pos[agent])
    else:
        swarm.vel_dec[agent]= np.zeros((2))


def agent_iteration(START_TIME, swarm, agent):

    init(swarm,agent)

    get_neighbors(agent,swarm)
    
    # ====================================================================================
    #     OBSTACLES AVOIDANCE + UPDATE COVERAGE MAP (timestamps) + TARGET GRID SELECTION
    # ====================================================================================
    # Compute obstacle avoidance term AND update coverage map AND target grid selection
    
    # AUX LOCAL VARIABLES
    max_fitness = 0 # Variable for storing the max_fitness
    best_angle = 180 # Variable for storing the best angle for choosing optimum goal cell
    
    # Loop over entire coverage area
    for x in range(0,Constants.WIDTH): 
        for y in range(0,Constants.LENGTH):

            point = np.array((x,y)) # grid cell under consideration in current iteration
            dist_to_point = norm2(swarm.pos[agent],point) # distance agent <--> grid_cell

            # ---------- OBSTACLE AVOIDANCE ----------
            # Obstacles are marked with NEG_INF in coverage map
            # If obstacles are inside D_0 radius, repulsion
            if swarm.coverage_map[agent][x][y] == Constants.NEG_INF: 
                swarm.vel_obs[agent] += (s(dist_to_point,Constants.D_O) * unitary_vector(point,swarm.pos[agent]))
                
            # ---------- UPDATE COVERAGE MAP ----------
            # If inside not an obstacle and is inside the sensor range, update timestamp
            elif dist_to_point < Constants.R_S: 
                if Constants.MODE=="continuous": swarm.coverage_map[agent][x][y] = time.monotonic()-START_TIME
                if Constants.MODE=="unique": swarm.coverage_map[agent][x][y] = 1
                
            # ---------- TARGET GRID SELECTION ----------
            # If not an obstacle and not inside radius, compute heuristics
            # We will perform target grid selection for those cells
            # that are:  R_S < cells < 2*R_S  
            elif (swarm.coverage_map[agent][x][y] != Constants.NEG_INF) and (not outside_area(x,y)):

                # Compute closest neighbor to this point
                closest = True
                for neig in swarm.neighbors[agent]:
                    dist_to_neig = norm2(point,swarm.pos[neig])
                    dist_to_neig_goal = norm2(point,swarm.goal[neig])
                    # If a neighbor is closer than agent (to point x,y)
                    # or if point is too close to neighbor goal --> skip point
                    if dist_to_neig<dist_to_point or dist_to_neig_goal < Constants.MIN_GOAL_DIST:
                        closest = False 

                if closest: # Compute fitness value for point

                    # ----------------- FITNESS FUNCTION CALCULATION -----------------
                    # Fitness value: the higher, the more priority
                    fitness = compute_fitness(swarm,agent,START_TIME,point,x,y,dist_to_point)
                    
                    if fitness > max_fitness:
                        max_fitness = fitness
                        swarm.goal[agent] = point # Update goal with point (x,y)
                        # Compute unitary velocity vector to goal
                        vel_goal=unitary_vector(swarm.pos[agent],point)
                        # Update best current angle to point to best current goal
                        best_angle = angle_between(swarm.vel_actual[agent],vel_goal)

                    # elif (fitness == max_fitness) and (fitness != 0):
                    # elif fitness == max_fitness:
                    elif abs(max_fitness-fitness)<0.005: # TODO: how to know this value
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
    decentering_velocity(swarm,agent)

    # ===================================================
    #                 SELFISHNESS TERM
    # ===================================================
    # Compute selfishness term with goal previously computed
    swarm.vel_sel[agent]=unitary_vector(swarm.pos[agent],swarm.goal[agent])

    # ===================================================
    #                 BOUNDARY REPULSTION TERM
    # ===================================================
    if outside_area(*swarm.pos[agent]):
        p_m = np.array((Constants.WIDTH/2,Constants.LENGTH/2))
        swarm.vel_bou[agent] = Constants.Q_M*unitary_vector(swarm.pos[agent],p_m)   
    
    # ===================================================
    #         FINAL VELOCITY DIRECTION TERM
    # ===================================================
    swarm.vel_desired[agent] = Constants.K_O*swarm.vel_obs[agent] \
                            + Constants.K_C*swarm.vel_dec[agent] \
                            + Constants.K_S*swarm.vel_sel[agent] \
                            + Constants.K_B*swarm.vel_bou[agent]
    

    # ===================================================
    #             COMPUTE DIFFERENCE ANGLE 
    # ===================================================    
    # Compute angle between actual velocity and desired velocity terms
    # Positive or negative sign
    sign_result = math.copysign(1,np.cross(np.append(swarm.vel_actual[agent],0),np.append(swarm.vel_desired[agent],0))[2])
    # Final computation for agent angle
    swarm.diff_angle[agent] = angle_between(swarm.vel_desired[agent],swarm.vel_actual[agent])*sign_result


    # ===================================================
    #                  CONTROL INPUT
    # ===================================================
    if swarm.diff_angle[agent]>=0:
        swarm.control_input[agent] = min(Constants.W_MAX,Constants.K_W*swarm.diff_angle[agent])
    else:
        swarm.control_input[agent] = max(-Constants.W_MAX,Constants.K_W*swarm.diff_angle[agent])


    # ===================================================
    #         KINEMATIC LAWS - UPDATE POSITION
    # ===================================================
    # Heading angle (IMPORTANT!!! --> First we must update the heading angle)
    swarm.heading_angle[agent] = swarm.heading_angle[agent] + Constants.CONSTANT_VELOCITY*Constants.TIME_STEP*swarm.control_input[agent]

    # Update the actual velocity once the heading angle is updated
    swarm.vel_actual[agent] = np.array([math.cos(math.radians(swarm.heading_angle[agent])),math.sin(math.radians(swarm.heading_angle[agent]))]) 

    # With the new updated heading angle, we compute the next agent position
    # X Position
    swarm.pos[agent][0] = swarm.pos[agent][0] + Constants.CONSTANT_VELOCITY*Constants.TIME_STEP*math.cos(math.radians(swarm.heading_angle[agent]))
    # Y Position
    swarm.pos[agent][1] = swarm.pos[agent][1] + Constants.CONSTANT_VELOCITY*Constants.TIME_STEP*math.sin(math.radians(swarm.heading_angle[agent]))

    
    # ============================ END OF AGENT ITERATION ===============================




def arguments():
    # -------------------------- Arguments parsing -------------------------- #
    # Options 
    options = "f:n:"
    # Long options 
    long_options = ["file=", "numuavs=", "realtime=", "trajectory=", "cumulative=", "temperature="]

    try:
        opts, args = getopt.getopt(sys.argv[1:],options,long_options)
    except getopt.GetoptError:
        # print('main.py -f <outputfile> -r <robot> -p <population_size> -t <tournament_size> -m <mutation_size> -e <pure_elitism>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            # print('main.py -f <outputfile> -h <help> -p <population_size> -t <tournament_size> -m <mutation_size> -e <pure_elitism>')
            sys.exit()
        elif opt in ("-f", "--file"):
            Constants.RESULTS_DIR = arg
        elif opt in ("-n", "--numuavs"):
            Constants.NUM_UAVS = int(arg)
        elif opt =="--realtime":
            Constants.REAL_TIME = bool(arg)

    if Constants.REAL_TIME==False: 
        Constants.TRAJECTORY_PLOT = False
        Constants.CUMULATIVE_PERCENTAGE = False
        Constants.COVERAGE_TEMPERATURE = False