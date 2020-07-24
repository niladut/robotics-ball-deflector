# Project : Ball Deflector

* Course Name: AI and Robotics SS20
* Authors: Ankit Dagar Niladri Dutta
* Date Created: June 2020

This repo is based on RAI code, including its python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.


## Index
  * [1. Objective](#1-objective)
    + [1.1 Primary Objective](#11-primary-objective)
      - [1.1.1 Environment Description](#111-environment-description)
      - [1.1.2 Behaviour Description](#112-behaviour-description)
    + [1.2 Secondary (Optional) Objective](#12-secondary--optional--objective)
      - [1.2.1 Environment Description](#121-environment-description)
      - [1.2.2 Behaviour Description](#122-behaviour-description)
    + [Task List](#task-list)
  * [2 Usage](#2-usage)

## 1. Objective
To demonstrate a simulation where the Panda Robot Arm is used to redirect the path of a moving (spherical) ball into a specified goal.
### 1.1 Primary Objective
To demonstrate a simulation where the Panda Robot Arm is used to redirect the path of a rolling (spherical) ball into a specified goal (cuboid hole).
#### 1.1.1 Environment Description
The simulation environment would contain a 2 Panda Robot Arms (A and B), an inclined plane (Ramp Type 1, see Figure 1 (a)), a deflector tool, two spheres (balls) and two cuboid holes (P and Q).
#### 1.1.2 Behaviour Description
1. Robot B shall lift the cylindrical stick from one end.
2. Robot A would lift a sphere and place it at the top of the inclined plane.
3. The sphere will roll down the ramp towards Robot B.
4. Robot B will hit the moving ball with the cylindrical stick, changing its direction such that it moves towards the target cuboid P.
5. The sphere rolls into the cuboid bin P.
6. The steps 1 to 5 are repeated, this time targeting cuboid bin Q.

Figure 1: Ramp Designs: (a) Ramp Type 1 (b) Ramp Type 2
 1

Figure 2: Top view of the environment setup. The dotted arrows indicate expected behaviour.
### 1.2 Secondary (Optional) Objective
To demonstrate a simulation where the Panda Robot Arm is used to redirect the path of a bouncing (spherical) ball into a specified goal (cuboid bin).
#### 1.2.1 Environment Description
The simulation environment would contain a 2 Panda Robot Arms (A and B), an inclined plane (Ramp Type 2, see Figure 1 (b)), a deflector tool, two spheres (balls) and two cuboid bins (P and Q).
In this scenario, Ramp Type 2 (Figure 1 (b)) will be used , causing the ball to bounce after leaving the ramp.
#### 1.2.2 Behaviour Description
1. Robot B shall lift the cylindrical stick from one end.
1. Robot A would lift a sphere and place it at the top of the inclined plane (Ramp Type 2).
1. The sphere will roll down the ramp and bounce towards Robot B.
1. Robot B will hit the bouncing ball with the deflector tool, changing the direction of the ball such that it moves towards the target cuboid bin P.
1. The sphere bounces into the cuboid bin P.
1. The steps 1 to 5 are repeated, this time targeting cuboid bin Q.


### Task List
The project consists of the following tasks which have to be completed in the duration of 6 weeks:
1. Environment Setup:
Setup the simulation environment according to Figure 2 .
1. Robot A:
    1. Pick and place spheres: For a spherical ball with a given(known) start position, pick and place it
at a specified target position.
    1. Perception of spherical balls using camera: Localise the pose of a spherical ball using image data from camera. Compare the calculated pose with the pose obtained from simulator directly.
    1. Integration A1: Use camera to perceive the position of a spherical ball, then pick and place it at a specified target position.
1. Robot B:
    1. Pick and move deflector tool: For the deflector tool with a given(known) start position, pick and
move it to a specified target position.
    1. Hit stationary ball to target: Using the deflector tool hit a stationary spherical ball at a given(known) position such that it moves towards a target position.
    1. Change direction of moving ball: Using the deflector tool hit a moving spherical ball at a given(known) position such that it changes direction(reflects) towards a target direction.
    1. Perception of deflector tool: Localise the pose of a deflector tool using image data from camera. Compare the calculated pose with the pose obtained from simulator directly.
    1. Perception of moving spherical ball: Localise the pose of a moving spherical ball using image data from camera. Compare the calculated pose with the pose obtained from simulator directly.
    1. Integration B1 (rolling ball): Use camera to perceive the position of a rolling spherical ball and use the deflector tool to deflect the ball such that it changes direction(reflects) towards a target direction.
    1. Change direction of bouncing ball: Using the deflector tool hit a bouncing spherical ball at a given(known) position such that it changes direction(reflects) towards a target direction.
    1. Integration B2 (bouncing ball): Use camera to perceive the position of a bouncing spherical ball and use the deflector tool to deflect the ball such that it changes direction(reflects) towards a target direction.


## 2. Usage
To run the project file
```bash
python3 src/main.py
```
