import math
import numpy as np

Kp = 1
Kd = 1
dt = 0.4 # sec
l = 0.2 # m, dist bt wheels
v = 0.5 # m/s, linear velocity constant 
SETPOINT = 2 # m, distance from wall 

def actuator(err, prev_err): 
    u = Kp * err 
    if prev_err: 
        u = Kp * err + Kd * (err - prev_err) / dt

    return u


def calculate_R_wdt(u): 
    R = v / u
    wdt = u * dt

    return R, wdt


def main(): 
    loc_orientation = np.array([[0], [0], [0]])   # x,y, theta as a column vector
    state = 1.5 
    error = 0.5 
    prev_err = 0 
    u = actuator(error, prev_err)
    i = 0 

    print(i, "@time=", round(i * dt,3), "-----"," state,error,u: ", round(state,3), round(error,3), round(u,3), "      x,y,theta: ", round(loc_orientation[0][0], 3), round(loc_orientation[1][0],3), round(loc_orientation[2][0],3) ) 
    
    while state != 2.0: 
    # while True: 
        i += 1

        R, wdt = calculate_R_wdt(u)
        # print( "    R, wdt: ", R, wdt)

        icc = np.array([loc_orientation[0][0] - R * math.sin(loc_orientation[2][0]), loc_orientation[1][0] + R * math.cos(loc_orientation[2][0])])
        
        # print("    icc: ", icc)

        mat1 = np.array([[math.cos(wdt), -math.sin(wdt), 0], [math.sin(wdt), math.cos(wdt), 0], [0, 0, 1]])
        mat2 = np.array([[loc_orientation[0][0] - icc[0]], [loc_orientation[1][0] - icc[1]], [loc_orientation[2][0]]])
        loc_orientation = mat1.dot(mat2) + np.array([[icc[0]], [icc[1]], [wdt]])

        state = 1.5 + loc_orientation[1][0]
        prev_err = error
        error = SETPOINT - state
        u = actuator(error, prev_err)

        print(i, "@time=", round(i * dt,3), "-----"," state,error,u: ", round(state,3), round(error,3), round(u,3), "      x,y,theta: ", round(loc_orientation[0][0], 3), round(loc_orientation[1][0],3), round(loc_orientation[2][0],3) ) 
    
        if i == 50:
            break 


main() 