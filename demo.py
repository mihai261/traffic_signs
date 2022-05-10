import cv2
import sys
import RPi.GPIO as GPIO
import os
import time
import pytesseract
from lcd import *

lcd_init()

os.environ['DISPLAY'] = ':0'

speed = 40
limit = 99

GPIO.setmode(GPIO.BCM)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(14,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.setup(17,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27,GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(14,GPIO.HIGH)


def inc_speed(arg):
    global speed
    if speed<=75:
        speed+=5
        
def dec_speed(arg):
    global speed
    if speed >=5:
        speed-=5
        
GPIO.add_event_detect(17, GPIO.FALLING, callback=dec_speed, bouncetime=500)
GPIO.add_event_detect(27, GPIO.FALLING, callback=inc_speed, bouncetime=500)

stopCascPath = sys.argv[1]
stopCascade = cv2.CascadeClassifier(stopCascPath)

limitCascPath = sys.argv[2]
limitCascade = cv2.CascadeClassifier(limitCascPath)

video_capture = cv2.VideoCapture(0)

while True:
    speedText = "Speed: " + str(speed);
    limitText = "Limit: " + str(limit);
    
    lcd_byte(LCD_LINE_1, LCD_CMD)
    lcd_string(speedText, 2)
    lcd_byte(LCD_LINE_2, LCD_CMD)
    lcd_string(limitText, 2)
    
    # Capture frame-by-frame
    GPIO.output(13,GPIO.LOW)
    GPIO.output(19, GPIO.LOW)
    
    if(speed > limit):
        GPIO.output(19, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(19, GPIO.LOW)
        
        time.sleep(0.1)
        
        GPIO.output(19, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(19, GPIO.LOW)

    ret, original_frame = video_capture.read()
    frame = cv2.rotate(original_frame, cv2.ROTATE_180)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    stops = stopCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    limits = limitCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    for (x, y, w, h) in stops:
        # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        GPIO.output(13,GPIO.HIGH)
    
    for (x, y, w, h) in limits:
        # cv2.rectangle(gray, (x+int(w/8), y+int(h/2)), (x+int(7*w/8), y+h), (0, 255, 0), 2)
        crop_img = gray[y+int(h/2):y+h, x+int(w/8):x+int(7*w/8)]
        # cv2.imshow('image', crop_img)
        text = pytesseract.image_to_string(crop_img, config='-l eng --oem 3 --psm 12')
        # print(text)
        numeric_filter = filter(str.isdigit, text)
        obs_limit = "".join(numeric_filter)
        
        if(len(obs_limit) == 2):
            limit = int(obs_limit)
            break

    # cv2.imshow('Video', gray)
    time.sleep(0.01)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
GPIO.output(13,GPIO.LOW)
GPIO.output(19,GPIO.LOW)
GPIO.cleanup()
cv2.destroyAllWindows()
