# !/bin/python
import RPi.GPIO as GPIO
import time
import os

GPIO.setmode(GPIO.BOARD)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(40, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(8, GPIO.LOW);

def shutdown(arg):
    GPIO.output(8,GPIO.LOW)
    GPIO.cleanup()
    time.sleep(0.5)
    os.system("sudo shutdown -h now")

GPIO.add_event_detect(40, GPIO.FALLING, callback=shutdown, bouncetime=2000)

while 1:
    time.sleep(1)
