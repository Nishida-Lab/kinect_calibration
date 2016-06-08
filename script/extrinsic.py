import cv
import cv2
import numpy as np
import glob

def draw(img, corners):
    for i in corners :
        for j in i :
            cv2.circle(img,tuple(j), 3, (0,0,255), -1)
    
    # corner = tuple(corners[0].ravel())
    # img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    # img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    # img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

if __name__ == "__main__": 

    # Load previously saved data
    mtx = np.float64([[403.533846, 0.0, 153.470313],
                      [0.0, 399.684149, 113.18246],
                      [0.0, 0.0, 1.0]])
    dist = np.float64([0.08348799999999999, -0.196376, 0.00032, 0.001903, 0.0])

    row_num = 11
    column_num = 9
    box_size = 0.04
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((row_num*column_num,3), np.float32)
    objp[:,:2] = np.mgrid[0:row_num,0:column_num].T.reshape(-1,2)
    for x in range(row_num*column_num):
        for y in range(3) :
            objp[x][y] *= box_size
    
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

    cap = cv2.VideoCapture(0)

    while True:
        ret, img = cap.read()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(img, (row_num,column_num),None)
        if ret == True:
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            
            print "-----------------------"
            # Find the rotation and translation vectors.
            print "-----------------------"
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)
            roll =  np.zeros((3,3), np.float64)
            rvecs_cvmat = cv.fromarray(rvecs)
            roll_cvmat = cv.fromarray(roll)
            
            cv.Rodrigues2(rvecs_cvmat, roll_cvmat)
            roll = np.asarray(roll_cvmat)  
            # print np.linalg.det(roll)
            # print roll.dot(roll.T)
            print "============================"
            print "ROLL"
            print roll
            print "============================"
            print "TVECS"
            print tvecs
            
            # project 3D points to image plane
            draw(img, corners)
            # imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            # print imgpts

        cv2.imshow('img',img)

        k = cv2.waitKey(1) # wait 1ms
        if k == 27: # ESC finish
            break

    cap.release()
    cv2.destroyAllWindows()
