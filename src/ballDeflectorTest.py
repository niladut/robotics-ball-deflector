#!/usr/bin/env python

from ball_deflector import BallDeflector
from utils import *

def hitBallTest():
    M = BallDeflector()
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', 'ball3', 'R_bin_base')
    M.runSim(700)
    input('Done...')
    M.destroy()

def hitBallTestDebug():
    M = BallDeflector(perceptionMode='cheat', debug = True)
    M.setTarget('ball3')
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', 'ball3', 'R_bin_base')
    M.runSim(700)
    input('Done...')
    M.destroy()

def hitBallPerceptionTest():
    M = BallDeflector(perceptionMode='komo', debug = True)
    M.setTarget('ball3')
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', 'ball3', 'R_bin_base')
    M.runSim(700)
    input('Done...')
    M.destroy()

def gripperOrientaionTest():
    M = BallDeflector()
    ballPosition = [1, 0, .3]
    M.createBallFrame('testball',ballPosition, [0,0,0,1],[0,1,1,0.3])
    targetOffset = [0,0,0.62]
    degrees = 150 #-58.59447385472987
    targetOrientation = euler_to_quaternion(degrees*np.pi/180,0,-45*np.pi/180)
    M.moveGripperIK('B', 'testball', targetOffset, targetOrientation)
    input('Done...')
    M.destroy()

def pickAndPlaceTest():
    M = BallDeflector()
    M.pickAndPlace('A', 'ball2', "ramp_1")
    input('Done...')
    M.destroy()

def pickAndPlacePerceptionTest():
    M = BallDeflector(perceptionMode='komo', debug = True)
    M.setTarget('ball1')
    M.runSim(200)
    M.pickAndPlace('A', 'ball2', "ramp_1")
    input('Done...')
    M.destroy()

def perceptionTest():
    ballFrame = "ball1"
    M = BallDeflector(perceptionMode='komo', debug = True)
    M.setTarget(ballFrame)
    M.runSim(200)
    M.testPerception(ballFrame,50)
    input('Done...')
    M.destroy()

def fullScenePerceptionTest():
    ballFrame = "ball1"
    M = BallDeflector(perceptionMode='komo', debug = True)
    M.setTarget(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', 'ball3', 'R_bin_base')
    M.runSim(700)
    input('Done...')
    M.destroy()

def fullScenePerceptionTest():
    ballFrame = "ball3"
    M = BallDeflector(perceptionMode='komo', debug = True)
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'R_bin_base')
    M.runSim(700)
    M.clearExtraFrames()
    input('Done...')
    M.runSim(100)
    M.destroy()

def fullScenePerceptionTest2():
    M = BallDeflector(perceptionMode='komo', debug = True)
    ballFrame = "ball3"
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'R_bin_base')
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
    M.runSim(100)
    input('Done...')
    M.destroy()

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

def runSimTest():
    ballFrame = "ball3"
    M = BallDeflector(perceptionMode='komo', debug = True)
    M.selectBall(ballFrame)
    M.runSim(200)
    M.clearExtraFrames()
    M.runSim(100)
    input('Done...')
    M.destroy()

def exportVideoTest():
    ballFrame = "ball3"
    M = BallDeflector(perceptionMode='komo', exportVideoMode = True, debug = True)
    input('Start...')
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'G_bin_base')
    M.runSim(700)
    M.clearExtraFrames()
    M.runSim(100)
    input('Done...')
    M.destroy()


def exportVideoTest():
    ballFrame = "ball3"
    M = BallDeflector(perceptionMode='komo', exportVideoMode = True, debug = True)
    input('Start...')
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'G_bin_base')
    M.runSim(700)
    M.clearExtraFrames()
    M.runSim(100)
    input('Done...')
    M.destroy()

def singleBallScene():
    ballFrame = "ball3"
    M = BallDeflector(perceptionMode='komo', exportVideoMode = True, debug = True)
    input('Start...')
    M.selectBall(ballFrame)
    M.runSim(200)
    M.pickAndPlace('A', ballFrame, "ramp_1")
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', ballFrame, 'G_bin_base')
    M.runSim(700)
    M.clearExtraFrames()
    M.runSim(100)
    input('Done...')
    M.destroy()

def main():
    # hitBallTest()
    # hitBallTestDebug()
    # hitBallPerceptionTest()
    # gripperOrientaionTest()
    # pickAndPlaceTest()
    # pickAndPlacePerceptionTest()
    # perceptionTest()
    # fullScenePerceptionTest()
    # fullScenePerceptionTest2()
    # fullScenePerceptionWithVideo()
    singleBallScene()
    # runSimTest()
    # exportVideoTest()

if __name__ == "__main__":
    main()
