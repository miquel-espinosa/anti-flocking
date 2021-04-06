"""
    Constants are defined here
"""

LENGTH=50
WIDTH=50
GEO_FENCE_WIDTH=3
NUM_UAVS=4

# TODO: Needs to be tweaked
# Weights for velocity parameters
K_O= 0.8 # obstacle and neighbor avoidance
K_C= 0.4 # decentering
K_S= 0.5 # selfishness
K_B= 0.1 # boundary control
K_W= 0.8 # weight for control input update

RHO= 0.2
ALPHA= 0.04
BETA= 0.01
Q_M= 0.6
Q_B= 0.8

CONSTANT_VELOCITY= 3
TIME_STEP = 0.2

# Communication range 
R_C = 15

# Sensor-perception range 
R_S = 5

# Max heading angle change
W_MAX=45

# SAFETY DISTANCE: distance threshold for s(z,d) function
D_O = 5 # For obstacle avoidance --> emergency situation
# Decentering value should take approximately 2*R_S (twice the sensor range)
D_C = 9 # For decentering term --> neighbor avoidance

# TODO: It needs to be adjusted correctly
# Negative infinite for marking obstacles in coverage map
NEG_INF = -10