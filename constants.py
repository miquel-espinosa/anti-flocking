"""
    Constants are defined here
"""

LENGTH=20
WIDTH=20
NUM_UAVS=1

# Weights for velocity parameters
K_O= 0.8 # obstacle avoidance
K_C= 0.4 # decentering
K_S= 0.5 # selfishness
K_B= 0.1 # boundary control
K_W= 0.4 # weight for control input update

RHO= 0.2
ALPHA= 0.04
BETA= 0.01
Q_M= 0.6
Q_B= 0.8

CONSTANT_VELOCITY= 5
TIME_STEP = 0.2

# Communication range 
R_C = 8

# Sensor-perception range 
R_S = 4

# Max heading angle change
W_MAX=60

# SAFETY DISTANCE: distance threshold for s(z,d) function
D_O = 4 # For obstacle avoidance and neighbor avoidance
D_C = 4 # For decentering term

# Negative infinite for marking obstacles in coverage map
NEG_INF = -100