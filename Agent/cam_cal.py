import numpy as np
import cv2, time

cv2.namedWindow('Image Feed')
cv2.moveWindow('Image Feed',159,-25)

cap = cv2.VideoCapture(0)

#setup camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
cap.set(cv2.CAP_PROP_FPS,40)

prev_frame_t = time.time()

cal_img_count = 0
frame_count = 0

while True:
    ret, frame = cap.read()

    #Process Calibration
    frame_count += 1
    if frame_count == 30:
        cv2.imwrite("cal_image_" + str(cal_img_count) + ".jpeg",frame)
        cal_img_count += 1
        frame_count = 0

    new_frame_t = time.time()
    fps = 1/(new_frame_t - prev_frame_t)
    prev_frame_t = new_frame_t
    cv2.putText(frame,'FPS ' + str(int(fps)), (10,40), cv2.FONT_HERSHEY_PLAIN, 3, (100,255,0), 2, cv2.LINE_AA)

    cv2.imshow('Image Feed',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()