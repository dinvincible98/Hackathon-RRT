import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.patches import Circle
from numpy import random
from math import sqrt

'''Using RRT to plan a path with obsticles'''

class Node():
    def __init__(self, position, parent, children):
        self.position = position           # x ,y
        self.parent = parent               # return index of nodes[] array
        self.children = children           # return list of indices of nodes[] array

def main():
    # Initial configuration
    q_init = Node((50,50),0, None)
    nodes = [q_init]                       # List of nodes
    delta = 1                              # Incremental distance
    Max = 100                              # Domain
    max_r = 12                             # Max radius
    # Draw random size of circles

    q_goal = Node((random.rand() * Max,random.rand() * Max), -1, 'end')       # random goal node 
    
    centers = []
    radii = []
    N = 25                                 # Number of circles
    K = 500                               # Number of nodes

    for i in range(N):
        while True:
            try:
                center = (random.rand() * Max, random.rand() * Max)
                radius = np.random.rand() *  max_r
                
                # Check collision  with initial node
                q_init_check = ((q_init.position[0] - center[0])**2 + (q_init.position[1] - center[1])**2)
                
                # Check collsion with goal node
                q_goal_check = ((q_goal.position[0] - center[0])**2 + (q_goal.position[1] - center[1])**2)
                
                # Collision
                if q_init_check <= radius**2 or q_goal_check <= radius**2:
                    raise Exception
            except:
                continue
            else:
                break
        
        centers.append(center)
        radii.append(radius)

    # Create a figure. Equal aspect so circles look circular
    fig,ax = plt.subplots(1)
    ax.set_aspect('equal')

    # Plot obstacles

    for  ci in range(len(centers)):
        circle = Circle((centers[ci][0],centers[ci][1]), radii[ci], color = "black")
        ax.add_patch(circle)

    for k in range(K):
        # Check if q_new is within an obstacles 
        while True:
            try:
                q_rand = (random.rand() * Max,random.rand() * Max)
                min_dist = np.inf                 # Set a a large minmum distance

                for n in range(len(nodes)):
                    new_dist = sqrt(((nodes[n].position[0] - q_rand[0])**2) + 
                                ((nodes[n].position[1] - q_rand[1])**2))
                    # Check dist
                    if new_dist < min_dist:
                        min_dist = new_dist

                        min_idx = n
                
                # Nearest node
                q_near = nodes[min_idx].position
                
                # q_rand to q_near in vector form
                q_vector = (q_rand[0]- q_near[0], q_rand[1]- q_near[1])

                #Vector
                q_N = np.array(q_near)
                q_V = np.array(q_vector)
                # Magnitude of vector
                mag = np.sqrt(np.dot(q_V,q_V)) 

                q_new = q_N + (delta / mag) * q_V

                # Check collision for new nodes:
                for j in range(len(centers)):
                    q_new_check = ((q_new[0] - centers[j][0])**2 + (q_new[1] - centers[j][1])**2)
                    if q_new_check <= radii[j]**2:
                        
                        raise Exception
                
            except:

                continue
            else:
                q_new = Node((q_new[0],q_new[1]), min_idx, None)          # Parent is q_near
                
                break

        q_latest = nodes[-1].position                                     # compare latest node to q_goal
        
        p1 = np.array(q_latest)
        p2 = np.array(q_goal.position)
        
        # Initial clear set to True
        clear = True
        # If clear is true after looping, then going to the goal 
        for m in range(len(centers)):
            
            p3 = np.array(centers[m])

            u = np.dot(p3-p1, p2-p1) / np.dot(p2-p1, p2-p1)

            p4 = np.array((p1[0] + u * (p2[0] - p1[0]), p1[1] + u * (p2[1] - p1[1])))

            d_intersect = np.sqrt(np.dot(p4-p3,p4-p3))

            if d_intersect <= radii[m] and 0 <= u <= 1:
                clear = False

                break
        
        # Replace q_new with q_goal if no obstacles
        if clear is True:
            q_goal.parent = -2
            q_new = q_goal

        # Add q_new to nodes if no obstacles    
        nodes.append(q_new)

        if clear is False:
            new_x = (q_near[0],q_new.position[0])
            new_y = (q_near[1],q_new.position[1])

            plt.plot(new_x,new_y, color = 'blue')
        else:
            new_x = (q_latest[0],q_new.position[0])
            new_y = (q_latest[1],q_new.position[1])
            plt.plot(new_x,new_y, color = 'red')
        
        plt.plot(q_init.position[0], q_init.position[1], 'go', markersize = 3)
        plt.plot(q_goal.position[0], q_goal.position[1], 'bo', markersize = 3)
        plt.xlim((0,100))            
        plt.ylim((0,100))
        plt.xlabel("X Range")
        plt.ylabel("Y Range")
        plt.title(str(k+1)+" Iterations")
        plt.pause(0.01)

        if clear is True:
            break
    
    # Draw the path   
    path = [q_goal.position]                                        # q_new = q_goal here
    q_next = Node(nodes[-2].position,nodes[-2].parent,None)         # Next node after q_goal

    while True:
        if q_next.position == q_init.position:
            path.append(q_next.position)
            break
        else:
            path.append(q_next.position)
            q_next = Node(nodes[q_next.parent].position,nodes[q_next.parent].parent, None)
    


    # Plot
    x_path = [i[0] for i in path]
    y_path = [i[1] for i in path]

    plt.plot(x_path,y_path, 'red', markersize = 2)
 
    plt.plot(q_init.position[0], q_init.position[1], 'bo', markersize = 3)
    plt.plot(q_goal.position[0], q_goal.position[1], 'go', markersize = 3)

    plt.show()



if __name__=="__main__":
    main()
            
