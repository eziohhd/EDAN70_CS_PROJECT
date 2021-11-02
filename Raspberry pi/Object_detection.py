import cv2
import numpy as np
import RPi.GPIO as GPIO

led = 14
tll = 15
cap = cv2.VideoCapture(0)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(14,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
GPIO.output(led, GPIO.HIGH)
GPIO.output(tll, GPIO.HIGH)
while True:
    _, frame = cap.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray_image, 80, 255, cv2.THRESH_BINARY)
    threshold=~threshold
    contours,_ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
       (x, y, w, h) = cv2.boundingRect(cnt)
       area=cv2.contourArea(cnt)
       
       if 50<area<200:
           cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
           cx = round((x + w) / 2)
           cy = round((y + h) / 2)
           print(cx,cy)
       
           cv2.putText(frame, '%d' % (cx), (cx,cy), 1, 1, (255, 255, 255), 2)
        
           cv2.putText(frame, '%d' % (cy), (cx+100,cy), 1, 1, (255, 255, 255), 2)
        
 
        
    cv2.imshow("Frame", frame)
  
    cv2.imshow("threshold", threshold)
    
    key = cv2.waitKey(1)
    if key == 27 :
        break

cap.release()
cv2.destroyAllWindows()
