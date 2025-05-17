import math
import numpy as np

def bound_coords(x_old, y_old, dx, dy, r,c): 
    x_new = x_old
    if 0 <= dx+x_old <= c-1: 
        x_new = dx+x_old
    
    y_new = y_old
    if 0 <= dy+y_old <= r-1: 
        y_new = dy+y_old
    
    return x_new, y_new

def get_neighbor_coords(x,y,dx,dy,r,c): 
    leftright = dy == 0 
    updown = dx == 0
    neighbors = []

    if updown and leftright: 
        return []  # action is 0,0 with no neighbors since probability is 1
    
    if updown: 
        neighbor_incr = [[-1, 0], [1,0]]
    else: # leftright 
        neighbor_incr = [[0, 1], [0,-1]]

    for ni in neighbor_incr: 
        nx, ny = bound_coords(x,y,ni[0], ni[1], r,c)
        neighbors.append([nx,ny])

    return neighbors

def update_states(states, rewards, success_probs, discount): 
    r, c = states.shape
    updated_states = np.zeros(states.shape)
    policy = np.empty(shape=states.shape, dtype=tuple) 

    # R L U D None
    actions = [[1,0], [-1,0], [0,1], [0,-1], [0,0]]
    action_reward = {}

    for x in range(c): 
        for y in range(r):
            # print("  cell: ", x,y)
            
            if (x, y) == (2, 0):  # termianl state with no possible actions 
                continue

            for dx,dy in actions: 
                ax, ay = bound_coords(x,y,dx,dy,r,c)
                neighbors = get_neighbor_coords(x,y, dx, dy, r, c) 

                # print("     action: ", dx,dy, "  // resulting cell: ", ax, ay)
                # print("        neighbors: ", neighbors)

                if dx == 0 and dy == 0: 
                    v = rewards[ay,ax] + discount * states[ay,ax]
                else: 
                    # process desired movement 
                    v = success_probs["target"] * (rewards[ay,ax] + discount * states[ay,ax])
                    # process undesired movement
                    for n in neighbors: 
                        v += success_probs["neighbor"] * (rewards[n[1],n[0]] + discount * states[n[1],n[0]])

                action_reward[(dx,dy)] = round(v, 5)
                # print("           v: ", action_reward[(dx,dy)])
            

            max_reward = -math.inf
            max_reward_action = None
            for action, reward in action_reward.items(): 
                # print(action, reward)
                if reward > max_reward: 
                    max_reward = reward 
                    max_reward_action = action 

            updated_states[y,x] = max_reward 
            policy[y,x] = max_reward_action

    return updated_states, policy


def value_iteration(states, rewards, num_iterations, success_probs, discount):
    for i in range(num_iterations): 
        print("ITERATION ", i+1)
        states, policy = update_states(states, rewards, success_probs, discount)
        print("updated value states: ")
        print_rows_flipped(states)
        print("updated policy")
        print_rows_flipped(policy)
        print(" ----- \n")

    return states, policy

def print_rows_flipped(nparr): 
    print(nparr[::-1])
    

states = np.zeros((2,3))
rewards = np.array([-0.1, -0.1, 1, -0.1, -0.1, -0.05]).reshape((2,3))
num_iterations = 3
discount = 0.9
success_probs = {"target": 0.9, "neighbor": 0.05}

print("initial state")
print_rows_flipped(states)
print(" \nrewards")
print_rows_flipped(rewards)
print(" ----- ")

value_iteration(states, rewards, num_iterations, success_probs, discount) 