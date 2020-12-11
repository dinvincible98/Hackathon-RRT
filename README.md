# Hackathon-RRT

## Overview

This is a RRT challenge in MSR Hackathon at Northwestern University. Rapidly-Exploring Random Tree (RRT) is a fundamental path planning algorithm in robotics. It consists of a set of vertices, which represent configurations in some domain D and edges, which connect two vertices. The algorithm randomly builds a tree in such a way that, as the number of vertices n  increases to inifinity, the vertices are uniformly distributed across the domain D in R^n. View the full description for this challenge [here](https://nu-msr.github.io/hackathon_site/rrt_challenge.html)
## RRT Pseudo Code

![Screenshot from 2020-12-10 20-38-48](https://user-images.githubusercontent.com/70287453/101854787-cfbd8700-3b27-11eb-8899-831e9534c849.png)

## Task

### Task1

Implement an RRT in a two-dimensional domain, D = [0,100] x [0,100]. Use an initial configuration of q_init = (50,50) and delta = 1.

Demo:

![Simple_RRT](https://user-images.githubusercontent.com/70287453/101854628-85d4a100-3b27-11eb-8850-4975f2b3603b.gif)









![NU_RRT](https://user-images.githubusercontent.com/70287453/101854626-853c0a80-3b27-11eb-99be-98e928137361.gif)
![obstacles_RRT](https://user-images.githubusercontent.com/70287453/101854627-85d4a100-3b27-11eb-8882-f27549fc37db.gif)
