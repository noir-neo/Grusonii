#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import signal
import sys

GPIO.setmode(GPIO.BCM)

def centering(gpionum, centerdc, maxdc, mindc):
    print "Centering a servo GPIO {0}".format(gpionum)

    GPIO.setup(gpionum, GPIO.OUT)
    servo = GPIO.PWM(gpionum, 50)
    servo.start(0.0)

    servo.ChangeDutyCycle(centerdc)
    time.sleep(0.5)
    servo.ChangeDutyCycle(maxdc)
    time.sleep(0.5)
    servo.ChangeDutyCycle(centerdc)
    time.sleep(0.5)
    servo.ChangeDutyCycle(mindc)
    time.sleep(0.5)
    servo.ChangeDutyCycle(centerdc)
    time.sleep(0.5)

    servo.stop()

centering(18, 7, 10, 4)
centering(12, 6, 7, 5)

GPIO.cleanup()
sys.exit(0)
