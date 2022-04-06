import cv2
import sys
import RPi.GPIO as GPIO
import os
import time
os.environ['DISPLAY'] = ':0'

GPIO.setmode(GPIO.BCM)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)

stopCascPath = sys.argv[1]
stopCascade = cv2.CascadeClassifier(stopCascPath)

limitCascPath = sys.argv[2]
limitCascade = cv2.CascadeClassifier(limitCascPath)

video_capture = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    GPIO.output(13,GPIO.LOW)
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
        # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        GPIO.output(19,GPIO.HIGH)

    # cv2.imshow('Video', frame)
    time.sleep(0.01)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
GPIO.output(13,GPIO.LOW)
GPIO.output(19,GPIO.LOW)
GPIO.cleanup()
cv2.destroyAllWindows()
