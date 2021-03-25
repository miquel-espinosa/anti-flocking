import math
import numpy as np

def s(z,d):
    if z<0:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

def norm1(p):
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

def norm2(p1,p2):
    p = p1-p2
    return math.sqrt(p[0]*p[0]+p[1]*p[1])

def unitary_vector(p1,p2):
    if (p1==p2).all():
        print("ERROR: p1 and p2 are equal")
        return p1
    return (p1-p2)/norm2(p1,p2)

