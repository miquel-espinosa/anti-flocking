"""
    Constants are defined here
"""
class Constants(object):

    LENGTH=50
    WIDTH=50
    GEO_FENCE_WIDTH=3
    NUM_UAVS=2

    # Weights for velocity parameters
    K_O= 0.8 # obstacle and neighbor avoidance
    K_C= 0.4 # decentering
    K_S= 0.5 # selfishness
    K_B= 0.1 # boundary control
    K_W= 0.8 # weight for control input update

    # -------- FITNESS FUNCTION PARAM --------
    RHO= 0.2
    ALPHA= 0.04 # Weight for distance: UAV <--> point
    BETA= 0.01 # Weight for distance: previous_goal <--> point

    # -------- BOUNDARY CONTROL PARAM --------
    Q_M= 0.6 # Weight for center of mission area
    Q_B= 0.8 # Weight for nearest boundary point

    CONSTANT_VELOCITY = 3 # In meters per second
    TIME_STEP = 0.2 # In seconds

    # Communication range 
    R_C = 15

    # Sensor-perception range 
    R_S = 5 

    # Max heading angle change
    W_MAX = 45

    # SAFETY DISTANCE: distance threshold for s(z,d) function
    D_O = R_S # For obstacle avoidance given an emergency situation
    D_N = 2 # For neighbor avoidance given an emergency situation
    # Decentering value should take approximately 2*R_S (twice the sensor range)
    D_C = 2*R_S # For decentering term --> neighbor avoidance threshold

    # Negative infinite for marking obstacles in coverage map
    NEG_INF = -40


    """
        COVERAGE MODE
         + unique: Unique coverage (no time consideration) E.g. agricultural coverage only once
         + continuous: Surveillance coverage (time consideration) E.g. surveillance of an industrial compound
    """
    MODE = "continuous"

    """ 
        Gives priority to those goals that will maximize coverage in a radius R_S.
        (It still needs a bit of readjusting with the fitness function)
    """
    GOAL_OPTIMIZATION = True

    """
        Minimum goal distance that must be ensured between agent goal and neighbor goal. Again, this avoids both UAVs moving towards same uncovered areas.
    """
    MIN_GOAL_DIST = 2*R_S

    """
        This parameter will disable the maximum communication radius range. 
        Therefore, if enabled, all UAVs will have information of all other UAVs coverage areas.
        Result will be, obviously, more coherent and efficient; but less realistic 
    """
    ALWAYS_COMMUNICATION = True

    """
        If enabled, drone failures will be simulated by decreasing the number of UAVs at defined iterations.
    """
    SIMULATE_FAILURES = True

    """
        Define the maximum iterations for covering area
    """
    MAX_ITERATIONS = 400

    """
        Define the maximum coverage percentage
    """
    MAX_COVERAGE = 95
    


    # =======================================================================================================

    """ --------------------------------------------
                PLOTTING AND DRAWING PARAMETERS
        -------------------------------------------- """
    # Global var to disable all plots
    PLOTS = True
    # Directory to store the results
    RESULTS_DIR = "results/sim"

    # If set to false will leave in blank corresponding plot 
    # (May be used for rapid testing, since disabling plotting improves performance)
    TRAJECTORY_PLOT = True
    CUMULATIVE_PERCENTAGE = True
    COVERAGE_TEMPERATURE = True
    # This constant will plot the coverage map only for the first UAV (red trajectory), not the global map
    COVERAGE_FIRST = True
    COST = True

    # Will output a video of the simulation (needs ffmpeg installed)
    VIDEOS = True

    # Draw the Sensor Radius R_S circle in canvas
    CIRCLE_SENSOR = True
    # Draw the Communication Radius R_C circle in canvas
    CIRCLE_COMMUNICATION = False