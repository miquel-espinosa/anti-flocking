"""
    Constants are defined here
"""

LENGTH=2000
WIDTH=2000
NUM_UAVS=5

# Weights for velocity parameters
K_O= 0.8 # obstacle avoidance
K_C= 0.4 # decentering
K_S= 0.5 # selfishness
K_B= 0.1 # boundary control

RHO= 0.2
ALPHA= 0.04
BETA= 0.01
Q_M= 0.6
Q_B= 0.8

CONSTANT_VELOCITY= 18
TIME_STEP = 0.2

# Communication range 
R_C = 400

# Sensor-perception range 
R_S = 150

# Max heading angle change
W_MAX=90

# SAFETY DISTANCE: distance threshold for s(z,d) function
D_O = 2 # For obstacle avoidance and neighbor avoidance
D_C = 2 # For decentering term