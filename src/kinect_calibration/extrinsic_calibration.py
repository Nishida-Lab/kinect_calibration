#!/usr/bin/env python

import cv
import cv2
import numpy as np
import math
import glob

class ExtrinsicCalibration(object):
    def __init__(self, row, column, size, mtx, dist):
        # == Flag define ==
        self.CIRCLE     = 0 # Use Circle Grid
        self.CHECKER    = 1 # Use Checker Board
        # == Point Informations ==
        self.row        = row      # Row number of points
        self.column     = column   # Column number of points
        self.size       = size+size # Distans between points (unit is [m])
        # Criteria of Points
        self.criteria   = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Point numbers calculate from Row and Column numbers
        self.objp       = np.zeros((row*column,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:row,0:column].T.reshape(-1,2)
        for x in range(row*column):
            for y in range(3):
                self.objp[x][y] *= self.size
        # == Camera Informations ==
        if mtx is None :
            self.mtx = np.float64([[403.533846, 0.0, 153.470313],
                                   [0.0, 399.684149, 113.18246],
                                   [0.0, 0.0, 1.0]])
        else :
            self.mtx  = mtx  # Camera Matrix (3x3)
            
        if dist is None :
            self.dist = np.float64([0.08348799999999999, -0.196376, 0.00032, 0.001903, 0.0])
        else:
            self.dist = dist # Dist Coeffs (4,5 ~ 8 arguments)

        # == OpenCV arguments ==
        self.use_window_flg = False # Flag whether use window or not
        self.rvecs          = None  # Rotete Vector
        self.tvecs          = None  # Translation Vector
        self.points         = None  # Coordinate value of Points on the Image
        
    def __del__( self ):
        if self.use_window_flg :
            cv2.destroyAllWindows()

    def calibrate(self, img, flg):
        # Get Gray Image
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Get Feature Points
        if flg == self.CHECKER :
            ret, self.points = cv2.findChessboardCorners(gray, (self.row,self.column), flags = cv2.CALIB_CB_FAST_CHECK)
            
        else :
            ret, self.points = cv2.findCirclesGridDefault(gray, (self.row,self.column))

        if ret :
            # Get Sub Pixel
            cv2.cornerSubPix(gray,self.points,(11,11),(-1,-1),self.criteria)
            # Get Pose & Positionret, points, rvecs, tvecs = ex_calib.calibrate(img, ex_calib.C
            self.rvecs, self.tvecs, self.inliers = cv2.solvePnPRansac(self.objp, self.points, self.mtx, self.dist)

            # Get 3x3 Rotation Matrix
            self.rotmx, jac, = cv2.Rodrigues(self.rvecs)
            # Get eular from Rotation Matrix
            self.roll, self.pitch, self.yaw = self.rotationMatrixToEulerAngles(self.rotmx)
        return ret

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self, R) :
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
        singular = sy < 1e-6
 
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
 
        return x, y, z

    def imgShow(self, img, ret):
        self.use_window_flg = True
        if ret :
            self.imgpts, jac = cv2.projectPoints(self.objp, self.rvecs, self.tvecs, self.mtx, self.dist)
            self.draw(img, self.points, self.imgpts)
        cv2.imshow('img', img)
        k = cv2.waitKey(1)

        return k

    def draw(self, img, points, imgpts):
        center = (self.row/2)*self.column+(self.column/2)
        cv2.line(img, tuple(imgpts[center].ravel()), tuple(imgpts[center-1].ravel()), (0,0,255), 3)
        cv2.line(img, tuple(imgpts[center].ravel()), tuple(imgpts[center+self.row].ravel()), (0,255,0), 3)
        for i in points :
            cv2.circle(img,tuple(i.ravel()), 3, (255, 0, 228), -1)
        for i in imgpts :
            cv2.circle(img,tuple(i.ravel()), 3, (0, 210, 255), -1)
if __name__ == "__main__":

    mtx = np.float64([[403.533846, 0.0, 153.470313],
                      [0.0, 399.684149, 113.18246],
                      [0.0, 0.0, 1.0]])
    dist = np.float64([0.08348799999999999, -0.196376, 0.00032, 0.001903, 0.0])
    
    ex_calib = ExtrinsicCalibration (5, 7, 0.035, mtx, dist)
    cap = cv2.VideoCapture(0)

    while True:
        ret, img = cap.read()
        ret  = ex_calib.calibrate(img, ex_calib.CIRCLE)
        k = ex_calib.imgShow(img, ret)
        if k == 27:
            break
        
    cap.release()
