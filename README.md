# Hackathon-RRT

## Overview

This is a RRT challenge in MSR Hackathon at Northwestern University. Rapidly-Exploring Random Tree (RRT) is a fundamental path planning algorithm in robotics. It consists of a set of vertices, which represent configurations in some domain D and edges, which connect two vertices. The algorithm randomly builds a tree in such a way that, as the number of vertices n  increases to inifinity, the vertices are uniformly distributed across the domain D in R^n. View the full description for this challenge [here](https://nu-msr.github.io/hackathon_site/rrt_challenge.html)
## RRT Pseudo Code

![Screenshot from 2020-12-10 20-38-48](https://user-images.githubusercontent.com/70287453/101854787-cfbd8700-3b27-11eb-8899-831e9534c849.png)

## Task

### Task1

Implement an RRT in a two-dimensional domain, D = [0,100] x [0,100]. Use an initial configuration of q_init = (50,50) and delta = 1.

Demo:

![Simple_RRT](https://user-images.githubusercontent.com/70287453/101856687-6b042b80-3b2b-11eb-80eb-82ca6c2f4510.gif)

### Task2

Implement RRT to plan a path when obstacles(Dark circles) is around. The result should indicate the start and end point, and a clear path without colliding obstacles.

Demo:

![Obstacles_RRT](https://user-images.githubusercontent.com/70287453/101856686-6b042b80-3b2b-11eb-92cf-8bc3f4028af7.gif)

### Task3

Import a NU picture and set the Letter 'N' as obstacles. Similar to task2, the program should return a clear path without colliding obstacles(Letter 'N') using RRT. The initial point is set to (40,40) and endpoint is (60,60).

Demo:

![NU_RRT](https://user-images.githubusercontent.com/70287453/101856685-6a6b9500-3b2b-11eb-9225-363474655e1f.gif)




