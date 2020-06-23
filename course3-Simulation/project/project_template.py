#!/usr/bin/env python

import sys
sys.path.append('../../build')
import libry as ry
import numpy as np
import time


def initRealWorld():
    print('##################### Configuring REAL WORLD ######################')
    #-- REAL WORLD configuration, which is attached to the physics engine
    RealWorld = ry.Config()
    RealWorld.addFile("../../scenarios/game_scene.g")

    # Configure Physics Engine
    S = RealWorld.simulation(ry.SimulatorEngine.physx, True)
    # Attach Camera Sensor
    S.addSensor("camera")
    camera = RealWorld.frame("camera")
    # input("Real World Setup Complete...")

    return [RealWorld, S, camera]



def initModelWorld():
    print('##################### Configuring MODEL WORLD ######################')
    #-- MODEL WORLD configuration, this is the data structure on which you represent
    # what you know about the world and compute things (controls, contacts, etc)
    C = ry.Config()
    #D = C.view() #rather use the ConfiguratioViewer below
    C.addFile("../../scenarios/game_scene.g")

    #-- using the viewer, you can view configurations or paths
    V = ry.ConfigurationViewer()
    V.setConfiguration(C)

    # input("Model World Setup Complete")
    return [C, V]

def endSimulation(S,V,C,RealWorld):
    print("Simulation Ended")
    S=0
    V=0
    C=0
    RealWorld=0

def pickPlaceBall(RealWorld, S, C, V, ball_frame):
    print('##################### Picking ',ball_frame,' ######################')
    A_gripper = C.frame("A_gripper")
    A_gripper.setContact(1)
    ball2 = C.frame(ball_frame)
    ball2.setContact(1)

    Xstart = C.getFrameState()

    tau = .01
    t = 0

    while True:
        t = t+1

        if t==500:
            break

        vel = np.zeros(C.getJointState().shape)
        S.step(vel, tau, ry.ControlMode.velocity)

    print('########################## ALIGNING #########################')
    T = 30
    C.setJointState(S.get_q())
    komo = C.komo_path(1.,T,T*tau,True)
    komo.addObjective([1.], ry.FS.positionDiff, ["A_gripperCenter", ball_frame], ry.OT.eq, [2e1], target=[0,0,0.1+0.03])
    komo.addObjective([1.], ry.FS.vectorZ, ["A_gripperCenter"], ry.OT.eq, scale=[1e1], target=[0,0,1]);
    komo.addObjective([1.], ry.FS.scalarProductYX, ["A_gripperCenter", ball_frame], ry.OT.eq);
    komo.addObjective([1.], ry.FS.scalarProductYY, ["A_gripperCenter", ball_frame], ry.OT.eq);
    komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
    komo.addObjective([], ry.FS.qItself, ["A_finger1"], ry.OT.eq, [1e1], order=1)
    komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
    komo.optimize()
    for t in range(T):
        C.setFrameState(komo.getConfiguration(t))
        q = C.getJointState()
        time.sleep(tau)
        S.step(q, tau, ry.ControlMode.position)
        V.setConfiguration(C)
        t += 1

    print('########################## ALIGNED! #########################')

    # while True:
    #     t = t+1
    #
    #     if t%5 == 0:
    #         [rgb, depth] = S.getImageAndDepth()
    #
    #     if t==2000:
    #         break
    #
    #     time.sleep(0.01)
    #     #vel = S.get_q()
    #     vel = np.zeros(C.getJointState().shape)
    #
    #
    #     if t<=600:
    #         [y,J] = RealWorld.evalFeature(ry.FS.positionDiff, ["A_gripperCenter", ball_frame])
    #         vel = J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ (-y)
    #         print(y)
    #     if t==600:
    #         print('##################### Grabbing ',ball_frame,' ######################')
    #         S.closeGripper("A_gripper")
    #     if t>=800:
    #         [y,J] = RealWorld.evalFeature(ry.FS.positionDiff, ["A_gripperCenter", "ball3"])
    #         vel = J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ (-y)
    #
    #
    # #     if S.getGripperIsGrasping("A_gripper"):
    # #         [y,J] = C.evalFeature(ry.FS.position, ["A_gripper"]);
    # #         q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ [0.,0.,-2e-4]
    #
    #     if t==900:
    #         print('##################### Releasing ',ball_frame,' ######################')
    #         S.openGripper("A_gripper")
    #
    # #     if S.getGripperIsGrasping("gripper"):
    # #         [y,J] = C.evalFeature(ry.FS.position, ["gripper"]);
    # #         q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ [0.,0.,-2e-4]
    #
    #     S.step(vel, tau, ry.ControlMode.velocity)

def deflectBall(ball_frame,bin_frame):
    print('##################### Deflecting',ball_frame,' to ',bin_frame,' ######################')


def pickDeflectorTool(delector_frame):
    print('##################### Picking ',delector_frame,' ######################')


def main():
    RealWorld, S, camera = initRealWorld()
    C, V = initModelWorld()

    # Robot B: Pick Deflector Tool
    pickDeflectorTool('deflector')


    # Robot A: Pick and Place Ball 2 on Ramp
    pickPlaceBall(RealWorld, S, C, V, 'ball2')

    # Robot B: Localize and deflect moving Ball to Target Q
    deflectBall('ball2','binQ')


    # Robot A: Pick and Place Ball 1 on Ramp
    pickPlaceBall(RealWorld, S, C, V, 'ball1')

    # Robot B: Localize and deflect moving Ball to Target P
    deflectBall('ball1','binP')

    # End Simulation
    endSimulation(S,V,C,RealWorld)

if __name__ == "__main__":
    main()
