#!/usr/bin/env python

import sys
sys.path.append('../../build')
import libry as ry
import numpy as np
import time

#-- REAL WORLD configuration, which is attached to the physics engine
# accessing this directly would be cheating!
RealWorld = ry.Config()
RealWorld.addFile("../../scenarios/game_scene.g")


S = RealWorld.simulation(ry.SimulatorEngine.physx, True)
S.addSensor("camera")

#input("Real World Setup Complete...")

#-- MODEL WORLD configuration, this is the data structure on which you represent
# what you know about the world and compute things (controls, contacts, etc)
C = ry.Config()
#D = C.view() #rather use the ConfiguratioViewer below
C.addFile("../../scenarios/game_scene.g")


#-- using the viewer, you can view configurations or paths
V = ry.ConfigurationViewer()
V.setConfiguration(C)

input("Model World Setup Complete...Start Simulation?")

#-- the following is the simulation loop
tau = .01

for t in range(300):
    time.sleep(0.01)

    #grab sensor readings from the simulation
    q = S.get_q()
    if t%10 == 0:
            [rgb, depth] = S.getImageAndDepth()  #we don't need images with 100Hz, rendering is slow

    #some good old fashioned IK
    C.setJointState(q) #set your robot model to match the real q
    V.setConfiguration(C) #to update your model display
    [y,J] = C.evalFeature(ry.FS.position, ["R_gripper"])
    vel = J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ [0.,0.,-1e-1];

    #send velocity controls to the simulation
    S.step(vel, tau, ry.ControlMode.velocity)

print("Simulation Ended")
S=0
V=0
C=0
RealWorld=0





