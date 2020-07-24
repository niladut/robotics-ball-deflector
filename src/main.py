#!/usr/bin/env python

from ball_deflector import BallDeflector
from utils import *

def fullScenePerceptionWithVideo():
    M = BallDeflector(perceptionMode='komo', debug = True, exportVideoMode = True)
    input('Start...')
    ballFrame = "ball3"
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'R_bin_base')
    M.runSim(200)
    M.clearExtraFrames()


    ballFrame = "ball2"
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'G_bin_base')
    M.clearExtraFrames()


    ballFrame = "ball1"
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'R_bin_base')
    M.runSim(700)
    M.clearExtraFrames()
    input('Done...')
    M.destroy()

def main():
    fullScenePerceptionWithVideo()

if __name__ == "__main__":
    main()
