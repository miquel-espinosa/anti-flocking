"""
    Constants are defined here
"""
class Constants(object):

    LENGTH=50
    WIDTH=50
    GEO_FENCE_WIDTH=3
    NUM_UAVS=2

    # TODO: Needs to be tweaked
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

    # Original values
    # RHO= 0.2
    # ALPHA= 0.04
    # BETA= 0.01
    # Q_M= 0.6
    # Q_B= 0.8

    CONSTANT_VELOCITY = 3
    TIME_STEP = 0.2

    # Communication range 
    R_C = 15

    # Sensor-perception range 
    R_S = 5

    # Max heading angle change
    W_MAX = 45

    # SAFETY DISTANCE: distance threshold for s(z,d) function
    D_O = R_S # For obstacle avoidance --> emergency situation
    D_N = 2 # For neighbor avoidance --> emergency situation
    # Decentering value should take approximately 2*R_S (twice the sensor range)
    D_C = 2*R_S # For decentering term --> neighbor avoidance threshold

    # Negative infinite for marking obstacles in coverage map
    NEG_INF = -40



    """
        COVERAGE MODE
        unique: Unique coverage (no time consideration)
        continuous: Surveillance coverage (time consideration)
    """
    MODE = "continuous"
    GOAL_OPTIMIZATION=True
    MIN_GOAL_DIST=R_S


    """
        PLOTTING PARAMETERS
    """
    REAL_TIME = True
    TRAJECTORY_PLOT = True
    CUMULATIVE_PERCENTAGE = True
    COVERAGE_TEMPERATURE = True
    COST = False

    VIDEOS = False

    CIRCLE_SENSOR = True
    CIRCLE_COMMUNICATION = False

