import math
import numpy as np

def s(z,d):
    if z<0:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

def norm(p1,p2):
    return np.linalg.norm(p1-p2)

def unitary_vector(p1,p2):
    if (p1==p2).all():
        print("ERROR: p1 and p2 are equal")
        return p1
    return (p1-p2)/norm(p1,p2)

