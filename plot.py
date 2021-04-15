import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import time, colorsys
import numpy as np

from constants import Constants

def trajectory_patch(history_x, history_y, agent, agent_colors):
    string_path_data = [
                (mpath.Path.MOVETO, (history_x[agent][-2],history_y[agent][-2])),
                (mpath.Path.CURVE3, (history_x[agent][-2],history_y[agent][-2])),
                (mpath.Path.CURVE3, (history_x[agent][-1],history_y[agent][-1]))]

    codes, verts = zip(*string_path_data)
    string_path = mpath.Path(verts, codes)
    return mpatches.PathPatch(string_path, color=agent_colors[agent], lw=2)


def plot_coverage_temperature(swarm, START_TIME):
    fig_cov_temp, ax_cov_temp = plt.subplots()
    image_cov_temp = ax_cov_temp.imshow(swarm.coverage_map[0], cmap=plt.cm.RdBu, extent=(-3, 3, 3, -3), interpolation='bilinear')
    if Constants.MODE=="unique": image_cov_temp.set_clim(-1,1)
    if Constants.MODE=="continuous": image_cov_temp.set_clim(Constants.NEG_INF,time.monotonic()-START_TIME)
    fig_cov_temp.colorbar(image_cov_temp)

    return fig_cov_temp, ax_cov_temp, image_cov_temp

def plot_simulation_map(history_x,history_y):
    fig_trajectories, ax_trajectories = plt.subplots()
    sc = [[] for i in range(Constants.NUM_UAVS)]
    for i in range(Constants.NUM_UAVS):
        sc[i] = ax_trajectories.scatter(history_x[i],history_y[i], s=1)

    boundary = Rectangle((0,0), Constants.WIDTH, Constants.LENGTH, linewidth=1, edgecolor='black', facecolor="gainsboro")
    geo_fence = Rectangle((Constants.GEO_FENCE_WIDTH,Constants.GEO_FENCE_WIDTH), Constants.WIDTH-2*Constants.GEO_FENCE_WIDTH, Constants.LENGTH-2*Constants.GEO_FENCE_WIDTH, linewidth=0.5, edgecolor='grey', facecolor="White")
    ax_trajectories.add_patch(boundary)
    ax_trajectories.add_patch(geo_fence)

    ax_trajectories.set_xlim(-10, Constants.WIDTH+10)   
    ax_trajectories.set_ylim(-10, Constants.LENGTH+10)  
    ax_trajectories.set_aspect('equal', adjustable='box')

    return fig_trajectories, ax_trajectories

def draw_obstacles(obstacles, ax_trajectories):
    for obs in obstacles:
        width = obs.ru[0]-obs.ld[0]
        height = obs.ru[1]-obs.ld[1]
        rect = Rectangle((obs.ld[0], obs.ld[1]), width, height, linewidth=1, edgecolor='black', hatch="////", facecolor="lightgrey")
        # Add the patch to the Axes
        if Constants.TRAJECTORY_PLOT: ax_trajectories.add_patch(rect)

def assign_agent_colors():
    agent_colors = []
    for c in np.arange(0., 360., 360./Constants.NUM_UAVS):
        (r, g, b) = colorsys.hls_to_rgb(c/360., (50 + np.random.rand() * 10)/100., (90 + np.random.rand() * 10)/100.)
        agent_colors.append((r, g, b))
    return agent_colors

    