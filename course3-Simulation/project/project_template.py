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

    def setTarget(self, targetFrame):
        self.targetFrame = targetFrame
        #you can also change the shape & size
        self.targetObj = self.RealWorld.getFrame(self.targetFrame)
        self.targetObj.setContact(1)

        self.obj = self.C.addFrame(self.targetFrame)
        self.obj.setShape(ry.ST.sphere, [.05])
        self.obj.setColor([1,1,0,0.9])
        self.obj.setContact(1)

    def setupSim(self):
        #-- REAL WORLD configuration, which is attached to the physics engine
        self.RealWorld = ry.Config()
        self.RealWorld.addFile("../../scenarios/game_scene.g")


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

    def openGripper(self, gripper_frame):
        print('########################## OPENING ##########################')
        self.S.openGripper(gripper_frame)
        self.C.attach("world", self.targetFrame)
        for i in range(50):
            time.sleep(self.tau)
            self.S.step([], self.tau, ry.ControlMode.none)
            self.C.setJointState(self.S.get_q())
            self.V.setConfiguration(self.C)
            self.t += 1
        print('########################## OPENED! ##########################')

    def align(self):
        print('########################## ALIGNING #########################')
        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        komo.addObjective([1.], ry.FS.positionDiff, ["A_gripperCenter", self.targetFrame], ry.OT.eq, [2e1], target=[0,0,0.1+0.03])
        komo.addObjective([1.], ry.FS.vectorZ, ["A_gripperCenter"], ry.OT.eq, scale=[1e1], target=[0,0,1]);
        komo.addObjective([1.], ry.FS.scalarProductYX, ["A_gripperCenter", self.targetFrame], ry.OT.eq);
        komo.addObjective([1.], ry.FS.scalarProductYY, ["A_gripperCenter", self.targetFrame], ry.OT.eq);
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, ["A_finger1"], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            time.sleep(self.tau)
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1

        print('########################## ALIGNED! #########################')

    def pick(self):
        T = 20
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        target = self.C.getFrame("A_gripperCenter").getPosition()
        target[-1] -= 0.12
        komo.addObjective([1.], ry.FS.position, ["A_gripperCenter"], ry.OT.eq, [2e1], target=target)
        komo.addObjective([], ry.FS.quaternion, ["A_gripperCenter"], ry.OT.eq, scale=[1e1], order=1)
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, ["A_finger1"], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            time.sleep(self.tau)
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1
        self.S.closeGripper("A_gripper")
        print('########################## CLOSING ##########################')
        while True:
            time.sleep(self.tau)
            self.S.step([], self.tau, ry.ControlMode.none)
            self.C.setJointState(self.S.get_q())
            self.V.setConfiguration(self.C)
            self.t += 1
            if self.S.getGripperIsGrasping("A_gripper"):
                print('########################## CLOSED! ##########################')
                self.C.attach("A_gripper", self.targetFrame)
                return True
            if self.S.getGripperWidth("A_gripper")<-0.05 :
                print('########################## FAILED! ##########################')
                return False


    def moveToDest(self):
        print('########################## LIFTING ##########################')
        T = 30
        self.C.setJointState(self.S.get_q())
        komo = self.C.komo_path(1.,T,T*self.tau,True)
        target = self.C.getFrame("ramp_1").getPosition()
        target[-1] += 0.5
        target[0] -= 0.35
        # target[:2] = (np.random.rand(2)-.5)/3
        komo.addObjective([1.], ry.FS.position, ["A_gripperCenter"], ry.OT.eq, [2e1], target=target)
        komo.addObjective([], ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e1])
        komo.addObjective([], ry.FS.qItself, ["A_finger1"], ry.OT.eq, [1e1], order=1)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], order=1)
        komo.optimize()
        for t in range(T):
            self.C.setFrameState(komo.getConfiguration(t))
            q = self.C.getJointState()
            self.S.step(q, self.tau, ry.ControlMode.position)
            self.V.setConfiguration(self.C)
            self.t += 1
            time.sleep(self.tau)

    def moveToInit(self):
        print('########################## GOtoINIT ##########################')
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

    def destroy(self):
        self.S = 0
        self.C = 0
        self.V = 0

    def pickAndLift(self):
        self.openGripper("A_gripper")
        self.moveToInit()
        self.perception()
        self.align()
#         input()
        success = self.pick()
        if success:
            self.moveToDest()
            print('########################### DONE! ###########################')
#         input()

    def pickAndPlace(self,targetFrame):
        self.setTarget(targetFrame)
        self.openGripper("A_gripper")
        self.moveToInit()
        self.perception()
        self.align()
    #         input()
        success = self.pick()
        if success:
            self.moveToDest()
            self.openGripper("A_gripper")
            print('########################### DONE! ###########################')
    #         input()


    def deflectBall(self,ball_frame,bin_frame):
        print('##################### Deflecting',ball_frame,' to ',bin_frame,' ######################')


    def pickDeflectorTool(self, delector_frame):
        print('##################### Picking ',delector_frame,' ######################')

def main():
    M = BallDeflector(perceptionMode='cheat')

    # Robot A: Pick and Place Ball 1 on Ramp
    M.pickAndPlace("ball1")

    # Robot B: Localize and deflect moving Ball to Target P
    M.deflectBall('ball1','binP')

    # Robot A: Pick and Place Ball 2 on Ramp
    M.pickAndPlace("ball2")

    # Robot B: Localize and deflect moving Ball to Target Q
    M.deflectBall('ball2','binQ')
    input('Done...')
    M.destroy()

# def main():
#     RealWorld, S, camera = initRealWorld()
#     C, V = initModelWorld()
#
#     # Robot B: Pick Deflector Tool
#     pickDeflectorTool('deflector')
#
#
#     # Robot A: Pick and Place Ball 2 on Ramp
#     pickPlaceBall(RealWorld, S, C, V, 'ball2')
#
#     # Robot B: Localize and deflect moving Ball to Target Q
#     deflectBall('ball2','binQ')
#
#
#     # Robot A: Pick and Place Ball 1 on Ramp
#     pickPlaceBall(RealWorld, S, C, V, 'ball1')
#
#     # Robot B: Localize and deflect moving Ball to Target P
#     deflectBall('ball1','binP')
#
#     # End Simulation
#     endSimulation(S,V,C,RealWorld)

if __name__ == "__main__":
    main()
