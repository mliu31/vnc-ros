import numpy as np

def bound_coords(x_old, y_old, dx, dy, r,c): 
    x_new = x_old
    if 0 <= dx+x_old <= c: 
        x_new = dx+x_old
    
    y_new = y_old
    if 0 <= dy+y_old <= r: 
        y_new = dy+y_old
    
    return x_new, y_new

def get_neighbor_coords(x,y,dx,dy,r,c): 
    updown = dy != 0 
    leftright = dx != 0
    neighbors = []

    if not (updown and leftright): 
        return []  # action is 0,0 with no neighbors since probability is 1
    
    if updown: 
        neighbor_incr = [[-1, 0], [1,0]]
    else: # leftright 
        neighbor_incr = [[0, 1], [0,-1]]

    for ni in neighbor_incr: 
        nx, ny = bound_coords(x,y,ni[0], ni[1], r,c)
        neighbors.append([nx,ny])

    return neighbors

def update_states(states, rewards): 
    r, c = states.shape
    
    # R L U D None
    actions = [[1,0], [-1,0], [0,1], [0,-1], [0,0]]
    action_reward = {}

    for x in range(c): 
        for y in range(r):
            print("  cell: ", x,y)
            for dx,dy in actions: 
                ax, ay = bound_coords(x,y,dx,dy,r,c)
                neighbors = get_neighbor_coords(x,y, dx, dy, r, c) 

                print("     action: ", dx,dy, "  // resulting cell: ", ax, ay)
                print("        neighbors: ", neighbors)
                # new_coord = [, ]
            print("-------------------\n")





def value_iteration(states, rewards, num_iterations, discount):
    for i in range(num_iterations): 
        update_states(states, rewards)

    return states



def print_rows_flipped(nparr): 
    print(nparr[::-1])
    

states = np.zeros((2,3))
rewards = np.array([-0.1, -0.1, 1, -0.1, -0.1, -0.05]).reshape((2,3))
num_iterations = 1
discount = 0.9

print("initial state")
print_rows_flipped(states)
print(" \nrewards")
print_rows_flipped(rewards)
print(" ----- ")

value_iteration(states, rewards, num_iterations, discount) 