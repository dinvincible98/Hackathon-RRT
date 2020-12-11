import math
import matplotlib.pyplot as plt
import numpy as np
import imageio
import sys
from math import sqrt
''' This is a simple RRT generator'''
def main():
    q_init = (50,50)          # Initial configuration
    nodes = [q_init]          # List of nodes
    K = 0                     # Number of vertices in nodes
    Max = 100                 # Domain
    delta = 1                 # Incremental distance
    itr = 400                 # Iterations
    for K in range (itr):
        q_rand = (np.random.rand()*Max, np.random.rand()*Max)

        min_dist = 100000               # Set a large default value for minmum distance
        
        for i in range(len(nodes)):
            new_dist = sqrt((nodes[i][0] - q_rand[0])**2 + (nodes[i][1] - q_rand[1])**2)

            if new_dist < min_dist:
                min_dist = new_dist

                min_idx = i

        q_near = nodes[min_idx]               # Find nearest node
        q_vector = (q_rand[0]-q_near[0],q_rand[1]-q_near[1])       # q_rand to q_near in vector form

        q_V = np.array(q_vector)
        q_N = np.array(q_near)

        mag = np.sqrt(np.dot(q_V,q_V))          # magnitude of vector

        q_new = q_N + (delta/mag) * q_V         # Move to new node in direction of q vector by step detla
        nodes.append(q_new)                     # Add new nodes to list

        # Plot
        new_x = (q_near[0],q_new[0])
        new_y = (q_near[1],q_new[1])

        plt.plot(new_x,new_y, color = "darkviolet")
        plt.title(str(K+1)+" Iterations")
        plt.xlim((0,100))
        plt.ylim((0,100))
        plt.xlabel("X Range")
        plt.ylabel("Y Range")
        plt.pause(0.001)

    plt.show()


if __name__=="__main__":
    main()