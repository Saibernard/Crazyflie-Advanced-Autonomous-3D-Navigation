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

---

This summary elucidates the mathematical and coding foundations underpinning the significant aspects of this project.

