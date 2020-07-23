import numpy as np
import cv2 as cv
print(cv.__version__)

def segmentColorPixels(rgb, colorMode):
    rgb = cv.cvtColor(rgb, cv.COLOR_BGR2RGB)
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

    # Red Pixels
    if colorMode == 'r':
        mask1 = cv.inRange(hsv, (  0, 120, 70), ( 10, 255, 255))
        mask2 = cv.inRange(hsv, (170, 120, 70), (180, 255, 255))
        mask = mask1 + mask2
    # Green Pixels
    elif colorMode == 'g':
        mask = cv.inRange(hsv, (50, 120, 70), ( 86, 255, 255))
    # Blue Pixels
    elif colorMode == 'b':
        mask = cv.inRange(hsv, (100,150,0), ( 140, 255, 255))
    # input()
    return mask

def depthImageToPCL(depth, rgb, mask):
    mask_pixels = np.where(mask>0)
    pointcloud = np.empty((mask_pixels[0].shape[0], 3))
    pointcloud[:,0] = mask_pixels[1]  # x pixels
    pointcloud[:,1] = mask_pixels[0]  # y pixels
    pointcloud[:,2] = depth[mask_pixels[0], mask_pixels[1]]

    masked_rgb = rgb[mask_pixels]
    return pointcloud, masked_rgb


def pclToCameraCoordinate(pixel_points, cameraFrame, fxfypxpy):
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


def find_pixels(rgb, depth, cameraFrame, fxfypxpy, colorMode):
    mask = segmentColorPixels(rgb, colorMode)
    pixel_points, masked_rgb = depthImageToPCL(depth, rgb, mask)
    obj_points, rel_points = pclToCameraCoordinate(pixel_points, cameraFrame, fxfypxpy)
    return obj_points, rel_points, masked_rgb


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
