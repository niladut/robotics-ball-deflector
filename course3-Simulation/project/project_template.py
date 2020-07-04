#!/usr/bin/env python

import sys
sys.path.append('../../build')
import libry as ry
import numpy as np
import time
import cv2 as cv
print(cv.__version__)
def _segment_redpixels(rgb):
    rgb = cv.cvtColor(rgb, cv.COLOR_BGR2RGB)
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, (  0, 120, 70), ( 10, 255, 255))
    mask2 = cv.inRange(hsv, (170, 120, 70), (180, 255, 255))
    return mask1 + mask2


## execise 1-2
def _image_pointcloud(depth, rgb, mask):
    mask_pixels = np.where(mask>0)
    pointcloud = np.empty((mask_pixels[0].shape[0], 3))
    pointcloud[:,0] = mask_pixels[1]  # x pixels
    pointcloud[:,1] = mask_pixels[0]  # y pixels
    pointcloud[:,2] = depth[mask_pixels[0], mask_pixels[1]]

    masked_rgb = rgb[mask_pixels]
    return pointcloud, masked_rgb


def _meter_pointcloud(pixel_points, cameraFrame, fxfypxpy):
    x = pixel_points[:,0]
    y = pixel_points[:,1]
    d = pixel_points[:,2]
    rel_points = np.empty(np.shape(pixel_points))
    rel_points[:,0] =  d * (x-fxfypxpy[2]) / fxfypxpy[0]
    rel_points[:,1] = -d * (y-fxfypxpy[3]) / fxfypxpy[1]
    rel_points[:,2] = -d

    cam_rot = cameraFrame.getRotationMatrix()
    cam_trans = cameraFrame.getPosition()
    points = rel_points @ cam_rot.T + cam_trans

    return points, rel_points # (num_points, 3)


def find_redpixels(rgb, depth, cameraFrame, fxfypxpy):
    mask = _segment_redpixels(rgb)
    pixel_points, masked_rgb = _image_pointcloud(depth, rgb, mask)
    obj_points, rel_points = _meter_pointcloud(pixel_points, cameraFrame, fxfypxpy)
    return obj_points, rel_points, masked_rgb



class BallDeflector:
    def __init__(self, perceptionMode='cheat'):
        self.tau = 0.01
        self.t = 0

        self.setupSim()
        self.setupC()
        self.perceptionMode = perceptionMode
        if perceptionMode == 'komo': self.setupKomoPerception()

    def runSim(self, time_interval):
        print('===== Running Sim for ',time_interval ,' units =====')
        for i in range(time_interval):
            time.sleep(self.tau)
            self.S.step([], self.tau, ry.ControlMode.none)
            self.C.setJointState(self.S.get_q())
            self.V.setConfiguration(self.C)
            self.t += 1

    def setTarget(self, targetFrame):
        self.targetFrame = targetFrame
        #you can also change the shape & size
        self.targetObj = self.RealWorld.getFrame(self.targetFrame)
        self.targetObj.setContact(1)

        self.obj = self.C.addFrame(self.targetFrame)
        self.obj.setShape(ry.ST.sphere, [.05])
        self.obj.setColor([1,1,0,0.9])
        self.obj.setContact(1)

    def createTarget(self, targetFrame, targetPosition = [0,0,0], targetQuaternion = [0,0,0,1], color = [1,0,0,0.9]):
        self.targetFrame = targetFrame

        self.obj = self.C.addFrame(self.targetFrame)
        self.obj.setShape(ry.ST.sphere, [.05])
        self.obj.setColor(color)
        self.obj.setContact(1)
        self.obj.setPosition(targetPosition)
        self.obj.setQuaternion(targetQuaternion)
        self.V.setConfiguration(self.C)

    def setupSim(self):
        #-- REAL WORLD configuration, which is attached to the physics engine
        self.RealWorld = ry.Config()
        self.RealWorld.addFile("../../scenarios/game_scene.g")

        testBall = self.RealWorld.getFrame('ball3')
        # objectPosition = self.testBall.getPosition()
        # testBall.setPosition([1, 0, .5])

        # Configure Physics Engine
        self.S = self.RealWorld.simulation(ry.SimulatorEngine.physx, True)
        # Attach Camera Sensor
        self.S.addSensor("camera")

        # Record Initial Position of Robot A
        self.q0 = self.S.get_q()

    def setupC(self):
        #-- MODEL WORLD configuration, this is the data structure on which you represent
        # what you know about the world and compute things (controls, contacts, etc)
        self.C = ry.Config()
        self.C.addFile("../../scenarios/game_scene.g")

        self.C.delFrame("ball1")
        self.C.delFrame("ball2")
        self.C.delFrame("ball3")

        #-- using the viewer, you can view configurations or paths
        self.V = ry.ConfigurationViewer()
        self.V.setConfiguration(self.C)
        self.cameraFrame = self.C.frame("camera")

        #the focal length
        self.f = 0.895
        self.f = self.f * 360.
        self.fxfypxpy = [self.f, self.f, 320., 180.]
        self.V.setConfiguration(self.C)

    def setupKomoPerception(self):
        self.perceptionC = ry.Config()

        table = self.perceptionC.addFrame("table")
        table.setPosition(self.C.frame("table").getPosition())
        table.setShape(ry.ST.ssBox, self.S.getGroundTruthSize("table"))

        self.percepObj = self.perceptionC.addFrame("ball")
        self.percepObj.setShape(ry.ST.sphere, [.05])
        self.percepObj.setContact(1)
        self.perceptionC.makeObjectsFree(["ball"])
        self.percepObj.setQuaternion([1,1,0,0])


    def komoBoxPose(self,obj_points, num_batch = 5):
        num_obj = int(obj_points.shape[0]/num_batch)
#         permutation = np.random.permutation(obj_points.shape[0])
        permutation = np.arange(obj_points.shape[0])
        for b in range(num_batch):
            for o in range(num_obj):
                name = "pointCloud%i" % o
                obj = self.perceptionC.addFrame(name)
                obj.setShape(ry.ST.sphere, [.001])
                indices = permutation[o+num_obj*b]
                obj.setPosition(obj_points[indices])

            komo = self.perceptionC.komo_path(1.,1,self.tau,True)
            komo.clearObjectives()
            komo.add_qControlObjective(order=1, scale=1e3) # Prior
            komo.addSquaredQuaternionNorms(0., 1., 1e2)
            if not self.S.getGripperIsGrasping("A_gripper"):
                komo.addObjective([1.], ry.FS.distance, [self.targetFrame, "table"], ry.OT.eq, [1e2])
                komo.addObjective([], ry.FS.vectorY, [self.targetFrame], ry.OT.eq, [1e2], order=1)
            for o in range(num_obj):
                name = "pointCloud%i" % o
                komo.addObjective([1.], ry.FS.distance, [self.targetFrame, name], ry.OT.sos, [1e0], target = [.001]) # Likelihood
            komo.optimize()

            self.perceptionC.setFrameState(komo.getConfiguration(0))
            p_obj = self.percepObj.getPosition()
            q_obj = self.percepObj.getQuaternion()
            for o in range(num_obj):
                name = "pointCloud%i" % o
                self.perceptionC.delFrame(name)

        return p_obj, q_obj


    def perception(self):
        # grab sensor readings from the simulation & set the model object to percept
        [rgb, depth] = self.S.getImageAndDepth()
        if self.perceptionMode == 'cheat': #TOTAL CHEAT: grab the true position from the RealWorld
            objectPosition = self.targetObj.getPosition()
            objectQuaternion = self.targetObj.getQuaternion()
            self.obj.setPosition(objectPosition)
            self.obj.setQuaternion(objectQuaternion)
            self.V.setConfiguration(self.C)
            errPer = 0.
        elif self.perceptionMode == 'komo':
            obj_points, rel_points, masked_rgb = find_redpixels(rgb, depth, self.cameraFrame, self.fxfypxpy)
            self.cameraFrame.setPointCloud(rel_points, masked_rgb)
            self.V.recopyMeshes(self.C)
            errPer = np.inf
            if len(obj_points)>0:
                p_obj, q_obj = self.komoBoxPose(obj_points)
                errPer = np.abs(self.obj.getPosition()-self.targetObj.getPosition()).max() # only for print
                self.obj.setPosition(p_obj)
                self.obj.setQuaternion(q_obj)
                self.V.setConfiguration(self.C)
        else:
            print('perceptionMode was not defined well!!')
        return errPer

    def testPerception(self):
        for i in range(1000):
            errPer=self.perception()
            self.S.step([], self.tau, ry.ControlMode.none)
            print('t: {:.1f}, Perception Error: {:.3f}'.format(self.t*self.tau, errPer))
            print('t: {:.1f}, True: {}, Estimated: {}'.format(self.t*self.tau, self.targetObj.getPosition(), self.obj.getPosition()))
            self.t += 1

    def openGripper(self, robotName):
        gripperFrame = robotName + "_gripper"
        print('===== Opening =====')
        self.S.openGripper(gripperFrame)
        self.C.attach("world", self.targetFrame)
        for i in range(50):
            time.sleep(self.tau)
            self.S.step([], self.tau, ry.ControlMode.none)
            self.C.setJointState(self.S.get_q())
            self.V.setConfiguration(self.C)
            self.t += 1
        print('===== Done Opening =====')

    def align(self, robotName):
        gripperCenterFrame = robotName + "_gripperCenter"
        gripperFingerFrame = robotName + "_finger1"
        print('===== Aligning =====')
        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        komo.addObjective([1.], ry.FS.positionDiff, [gripperCenterFrame, self.targetFrame], ry.OT.eq, [2e1], target=[0,0,0.1+0.03])
        komo.addObjective([1.], ry.FS.vectorZ, [gripperCenterFrame], ry.OT.eq, scale=[1e1], target=[0,0,1]);
        komo.addObjective([1.], ry.FS.scalarProductYX, [gripperCenterFrame, self.targetFrame], ry.OT.eq);
        komo.addObjective([1.], ry.FS.scalarProductYY, [gripperCenterFrame, self.targetFrame], ry.OT.eq);
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, [gripperFingerFrame], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            time.sleep(self.tau)
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1

        print('===== Done Aligning =====')

    def alignDeflector(self, robotName):
        gripperCenterFrame = robotName + "_gripperCenter"
        gripperFingerFrame = robotName + "_finger1"
        print('===== Aligning =====')
        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        komo.addObjective([1.], ry.FS.positionDiff, [gripperCenterFrame, self.targetFrame], ry.OT.eq, [2e1], target=[0.01,0,0.8])
        komo.addObjective([1.], ry.FS.vectorZ, [gripperCenterFrame], ry.OT.eq, scale=[1e1], target=[0,0,1]);
        komo.addObjective([1.], ry.FS.scalarProductYX, [gripperCenterFrame, self.targetFrame], ry.OT.eq);
        komo.addObjective([1.], ry.FS.scalarProductYY, [gripperCenterFrame, self.targetFrame], ry.OT.eq);
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, [gripperFingerFrame], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            time.sleep(self.tau)
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1

        print('===== Done Aligning =====')

    def pick(self, robotName):
        gripperCenterFrame = robotName + "_gripperCenter"
        gripperFrame = robotName + "_gripper"
        gripperFingerFrame = robotName + "_finger1"
        T = 20
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        target = self.C.getFrame(gripperCenterFrame).getPosition()
        target[-1] -= 0.12
        komo.addObjective([1.], ry.FS.position, [gripperCenterFrame], ry.OT.eq, [2e1], target=target)
        komo.addObjective([], ry.FS.quaternion, [gripperCenterFrame], ry.OT.eq, scale=[1e1], order=1)
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, [gripperFingerFrame], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            time.sleep(self.tau)
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1
        self.S.closeGripper(gripperFrame)
        print('===== Closed =====')
        while True:
            time.sleep(self.tau)
            self.S.step([], self.tau, ry.ControlMode.none)
            self.C.setJointState(self.S.get_q())
            self.V.setConfiguration(self.C)
            self.t += 1
            if self.S.getGripperIsGrasping(gripperFrame):
                print('===== Done Closing =====')
                self.C.attach(gripperFrame, self.targetFrame)
                return True
            if self.S.getGripperWidth(gripperFrame)<-0.05 :
                print('===== Failed to Close =====')
                return False


    def moveToDest(self, robotName, targetPose):
        gripperCenterFrame = robotName + "_gripperCenter"
        gripperFingerFrame = robotName + "_finger1"
        # print('===== Lifting =====')
        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        # target[:2] = (np.random.rand(2)-.5)/3
        komo.addObjective([1.], ry.FS.position, [gripperCenterFrame], ry.OT.eq, [2e1], target=targetPose)
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, [gripperFingerFrame], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1
            time.sleep(self.tau)

        # print('===== Done Lifting =====')

    def moveToInit(self):
        print('===== Go to Initial Pose =====')
        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [2e1], target=self.q0)
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1
            time.sleep(self.tau)

        print('===== At Initial Pose =====')

    def destroy(self):
        self.S = 0
        self.C = 0
        self.V = 0

    def pickAndLift(self, robotName, targetFrame):
        self.setTarget(targetFrame)
        self.openGripper(robotName)
        self.moveToInit()
        self.perception()
        self.align(robotName)
#         input()
        success = self.pick(robotName)
        if success:
            self.moveToDest(robotName)
            print('====== Done ======')
#         input()

    def pickAndPlace(self, robotName, targetFrame):
        self.setTarget(targetFrame)
        self.openGripper(robotName)
        self.moveToInit()
        self.perception()
        self.align(robotName)
    #         input()
        success = self.pick(robotName)
        if success:
            targetPose = self.C.getFrame("ramp_1").getPosition()
            targetPose[-1] += 1
            targetPose[0] -= 0
            self.moveToDest(robotName,targetPose)
            self.openGripper(robotName)
            print('====== Done ======')
    #         input()


    def deflectBall(self, robotName, ballFrame,binFrame):
        # print(' Deflecting',ballFrame,' to ',ballFrame,' #')
        print('===== Hitting ',ballFrame,' with Robot ', robotName ,' =====')

        self.setTarget(ballFrame)
        self.moveToInit()
        self.perception()
        self.alignDeflector(robotName)
        targetPose = self.RealWorld.getFrame(ballFrame).getPosition()
        # targetPose[1] += -0.55
        targetPose[2] += 0.55
        # targetPose[1] += 0.5

        self.moveToDest(robotName,targetPose)

    def movingBallPerception(self, ballFrame, observeTime, type='cheat'):
        if(type == 'cheat'):
            p1 = self.RealWorld.getFrame(ballFrame).getPosition()
            self.runSim(observeTime)
            p2 = self.RealWorld.getFrame(ballFrame).getPosition()
        else:
            # TODO: Add OpenCV Ball Position Perception
            p1 = [0,0,0]
            p2 = [0,0,0]

        return p1,p2

    def hitBall(self, robotName, ballFrame, goalFrame):
        print('===== Hitting ',ballFrame,' =====')
        dt = 25
        steps = 12
        total_time = dt*steps
        observeTime = 1
        for i in range(1,steps+1):
            p1,p2 = self.movingBallPerception(ballFrame,observeTime)
            position = self.calculateFuturePosition(p1,p2, dt*i,observeTime)
            self.createTarget('ball_'+str(i),position,[0,0,0,1])

        p1,p2 = self.movingBallPerception(ballFrame,observeTime)
        position = self.calculateFuturePosition(p1,p2, dt*i,observeTime)
        self.createTarget('future_ball',position,color = [0,1,0,0.9])
        # self.createTarget('future_ball',[1, 0, .3],[0,0,0,1])
        # self.moveGripper('B','init',[0.3,0,0.62],[ -0.383, 0,0,0.924])
        goalPos = self.RealWorld.getFrame('bin_2').getPosition()
        goalPos[2] = 0.3
        self.createTarget('goal',goalPos,[0,0,0,1])
        startPosition = self.C.getFrame('future_ball').getPosition()
        goalPosition = self.C.getFrame('goal').getPosition()
        offset = 0.3
        initPosition, angle = self.calculateDeflectorInit('B',startPosition, goalPosition, offset)
        q = euler_to_quaternion(angle,0,0)
        print('Init Pos : ',initPosition, ' , quaternion: ',q)
        self.createTarget('init',initPosition,[0,0,0,1],[1,1,0,0.9])
        self.moveGripper('B','init',[0,0,0.62],q)

        self.runSim(total_time)
        self.moveGripper('B','future_ball',[0,0,0.62],q)
        # targetPose = self.C.getFrame(targetFrame).getPosition()
        # targetPose[-1] += -0.55
        # targetPose[0] -= 0
        # self.moveToDest(robotName,targetPose)
    def moveGripper(self, robotName, targetFrame, targetOffset, targetOrientation):
        gripperCenterFrame = robotName + "_gripperCenter"
        gripperFingerFrame = robotName + "_finger1"
        print('===== Move Robot ',robotName,' with gripper frame ', gripperCenterFrame, ' to targetFrame ',targetFrame)

        # self.setTarget(targetFrame)
        #
        # self.def = self.C.addFrame(targetFrame)
        # self.def.setShape(ry.ST.sphere, [.05])
        # self.def.setColor([1,1,0,0.9])

        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        komo.addObjective([1.], ry.FS.positionDiff, [gripperCenterFrame, targetFrame], ry.OT.eq, [2e1], target=targetOffset)
        komo.addObjective([1.], ry.FS.quaternion, [gripperCenterFrame], ry.OT.eq, target=targetOrientation)
        komo.addObjective([1.], ry.FS.vectorZ, [gripperCenterFrame], ry.OT.eq, scale=[1e1], target=[0,0,1]);
        # komo.addObjective([1.], ry.FS.scalarProductYX, [gripperCenterFrame, self.targetFrame], ry.OT.eq);
        # komo.addObjective([1.], ry.FS.scalarProductYY, [gripperCenterFrame, self.targetFrame], ry.OT.eq);
        # komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        # komo.addObjective([], ry.FS.qItself, [gripperFingerFrame], ry.OT.eq, [1e1], order=1)
        # komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            time.sleep(self.tau)
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1

        print('===== Done Moving =====')

    def calculateFuturePosition(self,p1,p2, timeInterval, observeTime = 10):
        # t = observeTime

        [vx,vy] = [(p2[0] - p1[0])/observeTime, (p2[1] - p1[1])/observeTime ]

        v_abs = np.sqrt(vx*vx + vy*vy)
        distance = v_abs * (timeInterval - observeTime)

        print('p1 : ',p1)
        print('p2 : ',p2)
        print('vx,vy : ',[vx,vy])
        print('v_abs : ',v_abs)
        print('distance : ',distance)

        angle = np.arctan2((p2[1] - p1[1]),(p2[0] - p1[0]))
        position = [0,0,0]
        position[0] = p2[0] + distance*np.cos(angle)
        position[1] = p2[1] + distance*np.sin(angle)
        position[2] = p2[2]

        print('angle : ',angle)
        print('future pos : ',position)

        return position

    def calculateDeflectorInit(self,robotName,startPosition, goalPosition, offset):
        angle = np.arctan2((startPosition[1] - goalPosition[1]),(startPosition[0] - goalPosition[0]))
        initPosition = [0,0,0]
        initPosition[0] = startPosition[0] + offset*np.cos(angle)
        initPosition[1] = startPosition[1] + offset*np.sin(angle)
        initPosition[2] = 0.3
        print('Init Pos : ',initPosition, ' , Ang: ',angle)
        return initPosition, angle

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def main():
    M = BallDeflector()
    M.runSim(200)
    # Test Arm Movement
    M.hitBall('B', 'ball3', 'bin_2')
    M.runSim(500)

    # M.setTarget('future_ball')
    # M.perception()
    # M.moveGripper('B','deflector',[0,0,0])
    # M.createTarget('future_ball',[1, 0, .3],[0,0,0,1])
    # # M.moveGripper('B','init',[0.3,0,0.62],[ -0.383, 0,0,0.924])
    # goalPos = M.RealWorld.getFrame('bin_2').getPosition()
    # goalPos[2] = 0.3
    # M.createTarget('bin_2',goalPos,[0,0,0,1])
    # startPosition = M.C.getFrame('future_ball').getPosition()
    # goalPosition = M.C.getFrame('bin_2').getPosition()
    # offset = 0.3
    # initPosition, angle = M.calculateDeflectorInit('B',startPosition, goalPosition, offset)
    # q = euler_to_quaternion(angle,0,0)
    # print('Init Pos : ',initPosition, ' , quaternion: ',q)
    # M.createTarget('init',initPosition,[0,0,0,1],[1,1,0,0.9])
    # M.moveGripper('B','init',[0,0,0.62],q)
    #
    # M.runSim(500)
    # M.moveGripper('B','future_ball',[0,0,0.62],q)


    # M.runSim(600)
    # M.setTarget('ball3')
    # M.perception()
    # M.moveGripper('B','ball3',[0,0,0.62],[ -0.383, 0,0,0.924])
    # Robot B: Pick Deflector Tool
    # M.pickAndPlace("B", "deflector")

    # Robot A: Pick and Place Ball 1 on Ramp
    # M.pickAndPlace("A","ball1")
    # M.runSim(300)

    # Robot B: Localize and deflect moving Ball to Target P
    # M.deflectBall('B','ball3','binP')
    # M.deflectBall('B','ball3','binP')
    # M.runSim(500)
    # M.hitBall("B", "ball1")
    #
    # # Robot A: Pick and Place Ball 2 on Ramp
    # M.pickAndPlace("A","ball2")
    #
    # # Robot B: Localize and deflect moving Ball to Target Q
    # M.deflectBall('ball2','binQ')
    # M.runSim(1000)
    input('Done...')
    M.destroy()

if __name__ == "__main__":
    main()
