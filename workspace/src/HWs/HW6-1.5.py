import numpy as np
from math import sqrt as sqrt

def camera_proj(world_points, c_T_w, K): 
    print(c_T_w)
    print(K)
    for p in world_points: 
        p = np.append(p, 1)
        p_px = K @ c_T_w @ p.T

        print(p[:3], " --> ", p_px)




world_points = [np.array([12,11,5]), np.array([8,11,5]), np.array([8,9,5]), np.array([12,9,5])]
c_T_w = np.array([
    [1/sqrt(2), 1/sqrt(2), 0, -20/sqrt(2)], 
    [-1/sqrt(2), 1/sqrt(2), 0, 0], 
    [0, 0, 1, 0], 
    [0, 0, 0, 1]
])

K = np.array([
    [2592/5.75*5, 0, 2592/2, 0], 
    [0, 1944/4.28*5, 1944/2, 0], 
    [0, 0, 1, 0]
])


camera_proj(world_points, c_T_w, K)