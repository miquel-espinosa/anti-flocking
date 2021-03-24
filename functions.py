import math
import numpy as np

def s(z,d):
    if z<0:
        return 0
    else:
        return 1+math.cos((math.pi*(z+d))/(2*d))

def norm(p1,p2):
    return np.linalg.norm(p1-p2)

def unitary_direction(p1,p2):
    return (p1-p2)/norm(p1,p2)

# def get_coords(positions):
#     '''This function strips out x and y data from a list of boid objects for plotting'''
#     x=[]
#     y=[]

#     for i in range(len(positions)):
#         x.append(positions[i][1])
#         y.append(positions[i][2])
#     return x,y