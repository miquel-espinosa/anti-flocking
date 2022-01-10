from os import close
from constants import Constants
import math
import numpy as np

def s(z,d):
    if z<0 or z>d:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

def norm2(p1,p2):
    """ Scalar distance from p1 to p2 (without direction)"""
    return np.linalg.norm(p1-p2)

def unitary_vector(p1,p2):
    """ Results in a unitary vector going from p1 to p2  [p1 --> p2] """
    if (p1==p2).all():
        # return a random velocity for each
        return np.random.uniform(low=-1,high=1, size=(2,))
    return (p2-p1)/norm2(p1,p2)

def angle_between(vec1,vec2):

    def norm1(p):
        """Scalar magnitude of a vector from (0,0)"""
        return math.sqrt(p[0]*p[0]+p[1]*p[1])

    """ Compute angle between vector1 and vector2 """
    divisor = norm1(vec1)*norm1(vec2)
    dot_product = np.dot(vec1,vec2)
    
    # Ensure divisor is not zero to avoid error in division
    if divisor == 0: return 0 
    value = dot_product/divisor

    # Ensure a value between -1 and 1 is passed to acos function
    if value > 1: value = 1 
    elif value < -1: value = -1
    
    # Final computation for angle between vectors
    return math.degrees(math.acos(value))

def outside_area(x,y):
    """ Function to check if point is outside the permitted area """
    if (x<Constants.GEO_FENCE_WIDTH) or (y<Constants.GEO_FENCE_WIDTH) or \
        (x>Constants.LENGTH-Constants.GEO_FENCE_WIDTH) or (y>Constants.WIDTH-Constants.GEO_FENCE_WIDTH):
        return True
    return False

def assign_target(agent_pos,targets):
    closest_target = None
    min_dist = 100000000
    for target in targets:
        dist = norm2([target.x,target.y],agent_pos)
        if dist < min_dist and target.agent == Constants.UNASSIGNED_TARGET:
            min_dist = dist
            closest_target = target
    return closest_target
