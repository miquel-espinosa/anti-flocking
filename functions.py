import math
import numpy as np

def s(z,d):
    if z<0:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

# Scalar magnitude of a vector from (0,0)
def norm1(p):
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

# Scalar distance from p1 to p2 (without direction)
def norm2(p1,p2):
    p = p1-p2 # Since we are squaring the terms: p1-p2 = p2-p1
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

# Results in a unitary vector going from p1 to p2  [p1 --> p2] 
def unitary_vector(p1,p2):
    if (p1==p2).all():
        print("ERROR: p1 and p2 are equal")
        return np.array([0,0])
    return (p2-p1)/norm2(p1,p2)

# Compute angle between vector1 and vector2
def angle_between(vec1,vec2):
    divisor = norm1(vec1)*norm1(vec2)
    dot_product = np.dot(vec1,vec2)
    # Final computation for angle between vectors
    return math.degrees(math.acos(dot_product/divisor))

