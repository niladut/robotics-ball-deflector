#!/usr/bin/env python

import sys
sys.path.append('../../build')
import libry as ry
import numpy as np
import time

#-- REAL WORLD configuration, which is attached to the physics engine
RealWorld = ry.Config()
RealWorld.addFile("../../scenarios/game_scene.g")

# Configure Physics Engine
S = RealWorld.simulation(ry.SimulatorEngine.physx, True)
# Attach Camera Sensor
S.addSensor("camera")
camera = RealWorld.frame("camera")

#input("Real World Setup Complete...")

#-- MODEL WORLD configuration, this is the data structure on which you represent
# what you know about the world and compute things (controls, contacts, etc)
C = ry.Config()
#D = C.view() #rather use the ConfiguratioViewer below
C.addFile("../../scenarios/game_scene.g")

#-- using the viewer, you can view configurations or paths
#V = ry.ConfigurationViewer()
#V.setConfiguration(C)

input("Model World Setup Complete...Start Simulation?")

A_gripper = C.frame("A_gripper")
A_gripper.setContact(1)
ball2 = C.frame("ball2")
ball2.setContact(1)

Xstart = C.getFrameState()

tau = .01
t = 0

while True:
    t = t+1

    if t%5 == 0:
        [rgb, depth] = S.getImageAndDepth()

    if t==2000:
        break

    time.sleep(0.01)
    #vel = S.get_q()
    vel = np.zeros(C.getJointState().shape)

    if t<=600:
        [y,J] = RealWorld.evalFeature(ry.FS.positionDiff, ["A_gripperCenter", "ball2"])
        vel = J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ (-y)
        print(y)
    if t==600:
        S.closeGripper("A_gripper")
    if t>=800:
        [y,J] = RealWorld.evalFeature(ry.FS.positionDiff, ["A_gripperCenter", "ball3"])
        vel = J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ (-y)


#     if S.getGripperIsGrasping("A_gripper"):
#         [y,J] = C.evalFeature(ry.FS.position, ["A_gripper"]);
#         q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ [0.,0.,-2e-4]



    if t==900:
        S.openGripper("A_gripper")

#     if S.getGripperIsGrasping("gripper"):
#         [y,J] = C.evalFeature(ry.FS.position, ["gripper"]);
#         q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ [0.,0.,-2e-4]

    S.step(vel, tau, ry.ControlMode.velocity)

print("Simulation Ended")
S=0
V=0
C=0
RealWorld=0
