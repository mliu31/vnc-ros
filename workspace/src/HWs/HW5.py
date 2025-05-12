import numpy as np

def create_forwardbackward_mats(): 
    forward_mat = np.zeros((10, 10))
    backward_mat = np.zeros((10, 10))

    probabilities = [0.25, 0.5, 0.25]

    for i in range(8):
        col = forward_mat[:, i]
        col[i:i+3] = probabilities   

        bi = 9-i  # 9
        bcol = backward_mat[:, bi]
        bcol[bi-2:bi+1] = probabilities

    edge_probs = [[0.25, 0.75], [1]]
    
    for i in range(2):
        probs = edge_probs[i]

        fi = 8 + i
        col = forward_mat[:, fi]
        col[fi:fi+len(probs)] = probs

        bi = 1 - i
        bcol = backward_mat[:, bi]
        bcol[:bi+1] = probs

    return forward_mat, backward_mat   

def main(): 
    f, b = create_forwardbackward_mats()

    initial_pose = np.full(10, 1/10)

    # 3 forward, 2 backwards 
    cmds = ["f", "f", "f", "b", "b"]
    for i, c in enumerate(cmds): 
        if c == "f": 
            initial_pose = f @ initial_pose
        else: 
            initial_pose = b @ initial_pose
        print("bel ", i+1, " :", initial_pose, "\n")  # already normalized 

    
main()