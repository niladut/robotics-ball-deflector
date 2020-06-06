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

input("Model World Setup Complete...")


S=0
V=0
C=0
RealWorld=0





