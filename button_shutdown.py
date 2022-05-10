# !/bin/python
import RPi.GPIO as GPIO
import time
import os

GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP) 

def shutdown(arg):
    GPIO.output(14,GPIO.LOW)
    GPIO.cleanup()
    time.sleep(0.5)
    os.system("sudo shutdown -h now")

GPIO.add_event_detect(21, GPIO.FALLING, callback=shutdown, bouncetime=2000)

while 1:
    time.sleep(1)
