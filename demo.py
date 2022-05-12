import cv2
import sys
import RPi.GPIO as GPIO
import os
import time
import pytesseract
from lcd import *

speed = 40
limit = 99

GPIO.setmode(GPIO.BOARD)
GPIO.setup(10,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)

pwmR = GPIO.PWM(10, 2000)
pwmG = GPIO.PWM(12, 2000)
pwmB = GPIO.PWM(16, 2000)

pwmR.start(0)
pwmG.start(0)
pwmB.start(0)

GPIO.setup(33,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(35,GPIO.OUT)

GPIO.setup(11,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(13,GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(8,GPIO.HIGH)

lcd_init()

def setColor(r, g, b):
    pwmR.ChangeDutyCycle(r)
    pwmG.ChangeDutyCycle(g)
    pwmB.ChangeDutyCycle(b)

def inc_speed(arg):
    global speed
    if speed<=75:
        speed+=5
        
def dec_speed(arg):
    global speed
    if speed >=5:
        speed-=5
        
GPIO.add_event_detect(11, GPIO.FALLING, callback=dec_speed, bouncetime=500)
GPIO.add_event_detect(13, GPIO.FALLING, callback=inc_speed, bouncetime=500)

stopCascPath = sys.argv[1]
stopCascade = cv2.CascadeClassifier(stopCascPath)

limitCascPath = sys.argv[2]
limitCascade = cv2.CascadeClassifier(limitCascPath)

yieldCascPath = sys.argv[3]
yieldCascade = cv2.CascadeClassifier(yieldCascPath)

video_capture = cv2.VideoCapture(0)

while True:
    isStops = False
    isYields = False
    
    speedText = "Speed: " + str(speed);
    limitText = "Limit: " + str(limit);
    
    lcd_byte(LCD_LINE_1, LCD_CMD)
    lcd_string(speedText, 2)
    lcd_byte(LCD_LINE_2, LCD_CMD)
    lcd_string(limitText, 2)
    
    if speed >= limit+10:
        setColor(100, 0, 0)
    elif speed >= limit:
        setColor(0, 0, 100)
    else:
        setColor(0, 100, 0)

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
    
    yields = yieldCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    for (x, y, w, h) in stops:
        isStops = True
        GPIO.output(33,GPIO.HIGH)
    
    for (x, y, w, h) in yields:
        isYields = True
        GPIO.output(35,GPIO.HIGH)
        
    
    if isStops:
        GPIO.output(33,GPIO.HIGH)
    else:
        GPIO.output(33,GPIO.LOW)
        
    if isYields:
        GPIO.output(35,GPIO.HIGH)
    else:
        GPIO.output(35,GPIO.LOW)
        
    
    for (x, y, w, h) in limits:

        crop_img = gray[y+int(h/2):y+h, x+int(w/8):x+int(7*w/8)]
        text = pytesseract.image_to_string(crop_img, config='-l eng --oem 3 --psm 12')
        numeric_filter = filter(str.isdigit, text)
        obs_limit = "".join(numeric_filter)
        
        if(len(obs_limit) == 2):
            limit = int(obs_limit)
            break

    time.sleep(0.01)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
GPIO.output(33,GPIO.LOW)
GPIO.output(35,GPIO.LOW)
GPIO.output(8,GPIO.LOW)
GPIO.cleanup()
