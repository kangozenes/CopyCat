import cv2
import serial
import time
face_cascade= cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
vid=cv2.VideoCapture(0)
ArduinoSerial=serial.Serial('com5',9600,timeout=0.1)
time.sleep(1)
while vid.isOpened():
    ret, kare= vid.read()
    kare=cv2.flip(kare,1)
    gri = cv2.cvtColor(kare,cv2.COLOR_BGR2GRAY)
    yüz= face_cascade.detectMultiScale(gri,1.1,6)
    for x,y,w,h in yüz:
        konum='X{0:d}Y{1:d}'.format((x+w//2),(y+h//2))
        ArduinoSerial.write(konum.encode('utf-8'))
        cv2.circle(kare,(x+w//2,y+h//2),2,(255,0,0),2)
        cv2.rectangle(kare,(x,y),(x+w,y+h),(255,0,0),3)
    cv2.putText(kare, 'Furkan YILDIRIM' ,(370,50),cv2.FONT_HERSHEY_COMPLEX,  1,(0,255,255),2,cv2.LINE_AA)
    cv2.putText(kare, str(konum) ,(20,50),cv2.FONT_HERSHEY_COMPLEX,  1,(0,255,255),2,cv2.LINE_AA)
    cv2.rectangle(kare,(640//2-30,480//2-30),(640//2+30,480//2+30),(255,255,255),3)
    cv2.imshow('img',kare)
    if cv2.waitKey(10)&0xFF== ord('a'):
        break
vid.release()
cv2.destroyAllWindows()