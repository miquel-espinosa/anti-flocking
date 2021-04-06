from constants import GEO_FENCE_WIDTH, LENGTH, WIDTH
import math
import numpy as np

def s(z,d):
    if z<0:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

def norm1(p):
    """Scalar magnitude of a vector from (0,0)"""
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

def norm2(p1,p2):
    """ Scalar distance from p1 to p2 (without direction)"""
    p = p1-p2 # Since we are squaring the terms: p1-p2 = p2-p1
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

def unitary_vector(p1,p2):
    """ Results in a unitary vector going from p1 to p2  [p1 --> p2] """
    if (p1==p2).all():
        # print("ERROR: p1 and p2 are equal")
        # return a random velocity for each
        return np.random.uniform(low=-1,high=1, size=(2,))
    return (p2-p1)/norm2(p1,p2)

def angle_between(vec1,vec2):
    """ Compute angle between vector1 and vector2 """
    divisor = norm1(vec1)*norm1(vec2)
    dot_product = np.dot(vec1,vec2)
    
    # Ensure a value between -1 and 1 is passed to acos function
    value = dot_product/divisor
    if value > 1: value = 1
    elif value < -1: value = -1
    
    # Final computation for angle between vectors
    return math.degrees(math.acos(value))

def outside_area(x,y):
    """ Function to check if point is outiside the permitted area """
    if (x<GEO_FENCE_WIDTH) or (y<GEO_FENCE_WIDTH) or \
        (x>LENGTH-GEO_FENCE_WIDTH) or (y>WIDTH-GEO_FENCE_WIDTH):
        return True
    return False