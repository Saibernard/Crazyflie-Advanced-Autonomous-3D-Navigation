# Crazyflie-Advanced-Autonomous-3D-Navigation
This repository showcases an autonomous navigation system for the Crazyflie 2.0 quadrotor. Implemented are robust path-finding algorithms, trajectory generation, and collision avoidance mechanisms, enabling smooth and safe flights in complex 3D environments.

project 1.4

# Advanced Autonomous Navigation System for Crazyflie 2.0 Quadrotor

## 1. Overview

Using the Crazyflie 2.0 quadrotor, I designed and implemented an advanced autonomous navigation system. This system integrates robust path planning methods with adaptive trajectory generation techniques to ensure optimal movement in complex 3D environments.

## 2. Quadrotor & Environmental Representation

The Crazyflie quadrotor is modeled as a sphere with a 0.25m radius, maneuvering through a 3D voxelized grid environment. This environment digitization uses specific voxel resolution, striking a balance between computation speed and precision.

## 3. Path Planning Algorithms

I employed path planning algorithms such as Dijkstra and A*. These algorithms use the formula:

f(n) = g(n) + h(n)


Where:
- `f(n)` represents the total cost of node `n`.
- `g(n)` denotes the cost to reach node `n` from the start.
- `h(n)` is a heuristic that estimates the cost from node `n` to the goal, enhancing the algorithm's efficiency.

## 4. Trajectory Generation & Smoothing

The trajectories obtained from the graph algorithms were polished to be suitable for real-world quadrotor flights. A polynomial trajectory was formulated as:

s(t) = a0 + a1t + a2t^2 + ... + an*t^n


To ensure smooth transitions between trajectory segments, minimum jerk trajectories were utilized:

J = ∫(t0 to tf) (d^3x(t)/dt^3)^2 dt


Where `J` is the jerk over the time interval `[t0, tf]`.

## 5. Collision Avoidance & Dynamics

The quadrotor's dynamic movements were represented by the kinematic equations:

v(t) = ds(t)/dt
a(t) = dv(t)/dt


Here, `v(t)` and `a(t)` represent velocity and acceleration respectively. A PID controller:


u(t) = Kpe(t) + Ki∫e(t) dt + Kd*de(t)/dt


Where `e(t)` is the error, was employed to make real-time adjustments ensuring the quadrotor moves without colliding.

## 6. Code Integration

Key modules such as `se3_control.py` (quadrotor control) and `graph_search.py` (path planning) were incorporated. The primary module, `world_traj.py`, was central in combining trajectory generation with environmental feedback, guaranteeing seamless obstacle navigation for the quadrotor.

![3D_Path3](https://github.com/Saibernard/Crazyflie-Advanced-Autonomous-3D-Navigation/assets/112599512/1ef46b50-c086-4450-9a1b-29050ada85f1)

![Position_vs_Time](https://github.com/Saibernard/Crazyflie-Advanced-Autonomous-3D-Navigation/assets/112599512/d55a2a79-9dff-4762-b8f1-002df6744ad4)

![A_Path,_Waypoints,_and_Trajectory](https://github.com/Saibernard/Crazyflie-Advanced-Autonomous-3D-Navigation/assets/112599512/7657b577-24b6-49c8-a5e6-4773c9f1b81e)


![output](https://github.com/Saibernard/Crazyflie-Advanced-Autonomous-3D-Navigation/assets/112599512/248d1a64-6fee-4c7d-884e-ec0e4ac0d183)




### Simulation:

The simulation of the quad can be found in this video[https://youtu.be/UEcTmigXg7M] .

### Actual testing:

The actual testing of the quadcopter in the crazyflie hardware can be found here[https://youtu.be/E2RCtFpBg54]




---

This summary elucidates the mathematical and coding foundations underpinning the significant aspects of this project.

