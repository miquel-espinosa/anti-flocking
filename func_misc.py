'''
 A Python implementation of
 @article{ganganath2016anti-flocking,
 title={Distributed Anti-Flocking Algorithms for Dynamic Coverage of Mobile Sensor Networks},
 author={Nuwan Ganganath and Chi-Tsun Cheng and Chi K. Tse},
 journal={IEEE Transactions on Industrial Informatics},
 year={2016},
 volume={12},
 number={5},
 pages={1795--1805},
 publisher={IEEE}}

Tested using Python 3.5.2

Original MATLAB implementation can be found at https://github.com/manganganath/distributed_anti-flocking
'''

import numpy as np
import colorsys

def squd_norm(z):
    
    return np.add(np.square(z[:, :, 0]), np.square(z[:, :, 1]))


def sigma_norm(z, efs):
    
    return (1.0/efs)*(np.sqrt(1.0+efs*z*z)-1.0)
    
    
def get_gap(r):
    
    num_rows = r.shape[0]
    gap = np.zeros((num_rows, num_rows, 2),  dtype=np.float32);

    for a in range(1, num_rows):
        for b in range(a):
            gap[a, b, :] = r[b, :] - r[a, :]

    gap[:, :, 0] = gap[:, :, 0] - gap[:, :, 0].T
    gap[:, :, 1] = gap[:, :, 1] - gap[:, :, 1].T

    return gap
    
    
def action_func(z, d, c1):
    
    z=z/d
    p_h = np.zeros(z.shape)
    i = z<=1
    p_h[i] = -(c1*0.5*np.pi/d)*np.sin(0.5*np.pi*(z[i]+1))
                
    return p_h
    
    
def update_individual_record(cell_map, map_pos, x, T, r_s):

    for r in range(x.shape[0]):
        other_dist = np.sqrt(np.square(x[r, 0]-map_pos[:, :, 0]) + np.square(x[r, 1]-map_pos[:, :, 1]))
        temp_map = cell_map[:, :, 2*r]
        temp_ind = cell_map[:, :, 2*r+1]
        temp_map[other_dist<=r_s] = T
        temp_ind[other_dist<=r_s] = r
        cell_map[:, :, 2*r] = temp_map
        cell_map[:, :, 2*r+1] = temp_ind
        
    return cell_map
    
    
def fuse_record(cell_map, nbr):

    for i in range(1, nbr.shape[0]):
        for j in range(i):
            if nbr[i, j] == 1:
                max_time = np.maximum(cell_map[:, :, 2*i], cell_map[:, :, 2*j])
                temp_cell_i = cell_map[:, :, 2*i+1]
                temp_cell_i[cell_map[:, :, 2*i] != max_time] = j
                cell_map[:, :, 2*i] = max_time
                cell_map[:, :, 2*j] = max_time
                cell_map[:, :, 2*i+1] = temp_cell_i
                cell_map[:, :, 2*j+1] = temp_cell_i
                
    return cell_map
    
    
def fuse_all_records(cell_map, num_agents, fused_scan_record):

    max_time = np.maximum(fused_scan_record[:, :, 0], cell_map[:, :, 0])
    temp_fused = fused_scan_record[:, :, 1]
    temp_fused[max_time!=fused_scan_record[:, :, 0]] = 1
    fused_scan_record[:, :, 1] = temp_fused
    fused_scan_record[:, :, 0] = max_time

    for i in range(2, num_agents*2, 2):
        max_time = np.maximum(fused_scan_record[:, :, 0], cell_map[:, :, i])
        temp_fused = fused_scan_record[:, :, 1]
        temp_fused[max_time!=fused_scan_record[:, :, 0]] = (i+1.0)/2.0
        fused_scan_record[:, :, 1] = temp_fused
        fused_scan_record[:, :, 0] = max_time

    return fused_scan_record


def get_colors(num_colors):
    
    colors=[]
    for c in np.arange(0., 360., 360./num_colors):
        (r, g, b) = colorsys.hls_to_rgb(c/360., (50 + np.random.rand() * 10)/100., (90 + np.random.rand() * 10)/100.)
        colors.append((r, g, b))
    return colors
