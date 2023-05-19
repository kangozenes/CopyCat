import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685

# Initialize PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

r1 = digitalio.DigitalInOut(board.D17)
l1 = digitalio.DigitalInOut(board.D22)
r1.direction = digitalio.Direction.OUTPUT
l1.direction = digitalio.Direction.OUTPUT

print("READY")
pca.channels[0].duty_cycle = 0x8fff  # Channel 0 for ENA
time.sleep(1)
print("OK")
pca.channels[0].duty_cycle = 0x0000 



def stop():
    r1.value = False
    l1.value = False

def right():
    pca.channels[4].duty_cycle = 0x8fff
    pca.channels[5].duty_cycle = 0xffff

def left():
    pca.channels[4].duty_cycle = 0xffff
    pca.channels[5].duty_cycle = 0x8fff

def speed_up():
    pca.channels[4].duty_cycle = 0xffff
    pca.channels[5].duty_cycle = 0xffff

# speed down behaves like backward
def speed_down():
    pca.channels[4].duty_cycle = 0x8fff
    pca.channels[5].duty_cycle = 0x8fff


# area from coordinates
def PolyArea(x,y):
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

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

        # print(area)

        cv2.drawFrameAxes(frame,camera_matrix, camera_distortion, rvec, tvec, 100)
        # tvec_str = "x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        # cv2.putText(frame, tvec_str, (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2, cv2.LINE_AA)

    else:
        # stopping motor
        stop()




    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q') : break

    GPIO.cleanup()

cap.release()
cv2.destroyAllWindows()