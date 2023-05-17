import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
from _thread import *

"""
In this version thread is tried in the case v2 fails
"""


os.chdir("C:/Users/kango/Desktop/EE493-494/Implementation/Codes/kendi denemem") # change to current path


import RPi.GPIO as GPIO # THIS MAY CHANGE!!!!
GPIO.setmode(GPIO.BOARD)


# PIN ASSIGNMENT
L_Forward = 35
L_Backward = 36
R_Forward = 37
R_Backward = 38

GPIO.setup(L_Forward, GPIO.OUT)
GPIO.setup(L_Backward, GPIO.OUT)
GPIO.setup(R_Forward, GPIO.OUT)
GPIO.setup(R_Backward, GPIO.OUT)

def stop():
    GPIO.output(L_Forward, GPIO.LOW)
    GPIO.output(L_Backward, GPIO.LOW)
    GPIO.output(R_Forward, GPIO.LOW)
    GPIO.output(R_Backward, GPIO.LOW)

def right():
    GPIO.output(L_Forward, GPIO.HIGH)
    GPIO.output(L_Backward, GPIO.LOW)
    GPIO.output(R_Forward, GPIO.LOW)
    GPIO.output(R_Backward, GPIO.HIGH)

def left():
    GPIO.output(L_Forward, GPIO.LOW)
    GPIO.output(L_Backward, GPIO.HIGH)
    GPIO.output(R_Forward, GPIO.HIGH)
    GPIO.output(R_Backward, GPIO.LOW)

def speed_up():
    GPIO.output(L_Forward, GPIO.HIGH)
    GPIO.output(R_Forward, GPIO.HIGH)

# speed down behaves like backward
def speed_down():
    GPIO.output(L_Backward, GPIO.HIGH)
    GPIO.output(R_Backward, GPIO.HIGH) 

def drive():
    global area, edge, right_thr, left_thr, upper_area, lower_area, id 
    while not threadStop:
        if id is not None:
            if edge > right_thr:
                print("turn right")
                cv2.putText(frame, "turn right", (420,460), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA) # BGR
                right()
                time.sleep(0.015)

            elif edge < left_thr:
                print("turn left")
                cv2.putText(frame, "turn left", (420,460), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA)
                left()
                time.sleep(0.015)
            
            if area > upper_area:
                print("speed down")
                cv2.putText(frame, "speed down", (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)
                speed_down()
                time.sleep(0.025)
                
            elif area < lower_area:
                print("speed up")
                cv2.putText(frame, "speed up", (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)
                speed_up()
                time.sleep(0.025)
        else:
            # stopping motor
            stop()
    GPIO.cleanup()


# area from coordinates
def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

if __name__ == "__main__":

    global area, edge, right_thr, left_thr, upper_area, lower_area, id, threadStop
    threadStop = False
    marker_size = 100

    # calibration
    with open('cam_cal.npy', 'rb') as f:
        camera_matrix= np.load(f)
        camera_distortion = np.load(f)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    cap = cv2.VideoCapture(1)

    camera_width= 640
    camera_height= 480
    camera_frame_rate= 40

    cap.set(2, camera_width)
    cap.set(4, camera_height)
    cap.set(5, camera_frame_rate)

    start_new_thread(drive,()) # thread to drive the motor

    while True:

        ret, frame = cap.read()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, id, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

        if id is not None:

            aruco.drawDetectedMarkers(frame, corners)

            rvec_all, tvec_all, _objPts = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec = rvec_all[0][0]
            tvec = tvec_all[0][0]

            corners = corners[0][0]

            x = corners[:,0]
            y = corners[:,1]

            edge = x[0]
            
            area = PolyArea(x,y)

            # area thresholds
            upper_area = 50000
            lower_area = 35000

            # left/right thresholds
            left_thr = 180
            right_thr = 320

            # print(area)

            cv2.drawFrameAxes(frame,camera_matrix, camera_distortion, rvec, tvec, 100)
            # tvec_str = "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            # cv2.putText(frame, tvec_str, (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)
            

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            threadStop = True # terminating the thread
            break

    cap.release()
    cv2.destroyAllWindows()