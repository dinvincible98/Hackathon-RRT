import numpy as np
import math
import cv2 as cv
import matplotlib.pyplot as plt
from numpy import random, sqrt
from math import sqrt

'''This program runs RRT in a NU image'''

class Node():
    def __init__(self, position, parent, children):
        self.position = position           # x ,y
        self.parent = parent               # return index of nodes[] array
        self.children = children           # return list of indices of nodes[] array


def check(clear, q_latest, q_goal, img):
    '''Bresenham's line algorithm for checking collisions with straight lines'''
    x0 = int(np.floor(q_latest[0]))
    y0 = int(np.floor(q_latest[1]))
    x1 = int(np.floor(q_goal[0]))
    y1 = int(np.floor(q_goal[1]))

    if img[y1,x1] == 0:                 # Line and point check
        clear = False
    elif x1 - x0 == 0:                  # Delta x is 0 so line is vertical     
        y_increment = np.sign(y1 - y0)
        y = y0
        
        while y != y1:
            if img[y,x0] == 0:
                clear = False
                break
            y += y_increment
    elif y1 - y0 == 0:                  # Delta y is 0 so line is horizontal
        x_increment = np.sign(x1 -x0)
        x = x0

        while x != x1:
            if img[y0,x] == 0:
                clear = False
                break
            x += x_increment
    else:
        if abs(y1 - y0) < abs(x1-x0):
            if x0 > x1:
                dx = x0 - x1
                dy = y0 - y1
                y_increment = np.sign(dy)
                dy *= y_increment
                D = 2 * dy - dx
                y = y1

                for x in range(x1,x0):
                    if img[y,x] == 0:
                        clear = False
                        break
                    if D > 0:
                        y += y_increment
                        D = D - 2 * dx
                    D = D + 2 * dy
            else:
                dx = x1 - x0
                dy = y1 - y0
                y_increment = np.sign(dy)
                dy *= y_increment
                D = 2 * dy - dx
                y = y0

                for x in range(x0,x1):
                    if img[y,x] == 0:
                        clear = False
                        break
                    if D > 0:
                        y += y_increment
                        D = D - 2 * dx
                    D = D + 2 * dy
        else:
            if y0 > y1:
                dx = x0 - x1
                dy = y0 - y1
                x_increment = np.sign(dx)
                dx *= x_increment
                D = 2 * dx - dy
                x = x1

                for y in range(y1,y0):
                    if img[y,x] == 0:
                        clear = False
                        break
                    if D > 0:
                        x += x_increment
                        D = D - 2 * dy
                    D = D + 2 * dx
            else:
                dx = x1 - x0
                dy = y1 - y0
                x_increment = np.sign(dx)
                dx *= x_increment
                D = 2 * dx - dy
                x = x0

                for y in range(y0,y1):
                    if img[y,x] == 0:
                        clear = False
                        break
                    if D > 0:
                        x += x_increment
                        D = D - 2 * dy
                    D = D + 2 * dx
    return clear


def main():
    q_init = Node((40,40), 0, None)              # Initial configuration
    nodes = [q_init]                             # List of nodes
    delta = 2                                    # Incremental step
    Max = 100                                    # Domain
    
    #Load Image
    im = plt.imread('N_map.png')
    im = np.flipud(im)
    plt.imshow(im)

    obstacles = []                               # List of obstacles
    
    img = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    
    for i in range(Max):
        for j in range(Max):
            if img[i][j] == 0.0:
                config = [i, j]
                obstacles.append(config)

    q_goal = Node((60,60), -1, 'end')             # Goal node
    
    K = 1000     
    
    for  k in range(K):
        while True:
            try:
                q_rand = (random.rand() * Max, random.rand() * Max)
                min_dist = np.inf

                for node in range(len(nodes)):
                    new_dist = sqrt((nodes[node].position[0] - q_rand[0])**2 +
                                    (nodes[node].position[1] - q_rand[1])**2)

                    if new_dist < min_dist:
                        min_dist = new_dist

                        min_idx = node

                q_near =  nodes[min_idx].position                  # Nearest node position          
                q_vector = (q_rand[0] - q_near[0], q_rand[1] - q_near[1])

                q_V = np.array(q_vector)
                q_N = np.array(q_near)

                mag = sqrt(np.dot(q_V,q_V))
                q_new = q_N + (delta / mag) * q_V

                clear = check(True,q_N,q_new,img)

                if clear is False:
                    raise Exception
            
            except:
                continue
            
            else:
                q_new = Node((q_new[0], q_new[1]), min_idx, None)
                break

        q_latest = nodes[-1].position

        clear = check(True, q_latest, q_goal.position, img)

        if clear is True:
            q_goal.parent = -2
            q_new = q_goal
        
        nodes.append(q_new)

        if clear is False:
            new_x = (q_near[0],q_new.position[0])
            new_y = (q_near[1],q_new.position[1])
            plt.plot(new_x, new_y, color="darkviolet")
        else:
            new_x = (q_latest[0],q_new.position[0])
            new_y = (q_latest[1],q_new.position[1])
            plt.plot(new_x, new_y, color="yellow")

        plt.plot(q_init.position[0], q_init.position[1], 'bo', markersize = 4)
        plt.plot(q_goal.position[0], q_goal.position[1], 'go', markersize = 4)
        plt.xlim((0,100))            
        plt.ylim((0,100))
        plt.xlabel("X Range")
        plt.ylabel("Y Range")
        plt.title(str(k+1)+" Iterations")
        plt.pause(0.01)

        if clear is True:
            break

    # Draw path
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
 

    plt.show()
                          

if __name__=="__main__":
    main()