from constants import Constants
import math
import numpy as np

def s(z,d):
    if z<0 or z>d:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

def norm1(p):
    """Scalar magnitude of a vector from (0,0)"""
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

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
    """ Compute angle between vector1 and vector2 """
    divisor = norm1(vec1)*norm1(vec2)
    dot_product = np.dot(vec1,vec2)
    
    # Ensure a value between -1 and 1 is passed to acos function
    # if divisor == 0: return 0
    value = dot_product/divisor
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

def radius_covered(cov_map,pos):
    """ Function to check in a square of R_S*R_S the number of already covered cells """
    x0 = int(np.floor(pos[0]-Constants.R_S))
    y0 = int(np.floor(pos[1]-Constants.R_S))
    x0_lower = max(x0,0)
    y0_lower = max(y0,0)
    x_upper = min(x0+2*Constants.R_S,Constants.WIDTH) 
    y_upper = min(y0+2*Constants.R_S,Constants.LENGTH)

    total_covered=0
    for x in range(x0_lower,x_upper):
        for y in range(y0_lower,y_upper):
            if cov_map[x][y]!=0:
                total_covered = total_covered + 1
    return total_covered

def percentage_covered(swarm):
    """ Function to compute total percentage covered """
    area = Constants.WIDTH * Constants.LENGTH
    if Constants.NUM_UAVS >= 2:
        aux_max = np.maximum(swarm.coverage_map[0],swarm.coverage_map[1])
        for i in range(2,Constants.NUM_UAVS): aux_max = np.maximum(aux_max,swarm.coverage_map[i])
    else:
        aux_max = swarm.coverage_map[0]
    return (np.count_nonzero(aux_max)/area)*100 
