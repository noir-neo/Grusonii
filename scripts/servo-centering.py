#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import signal
import sys

GPIO.setmode(GPIO.BCM)

def changeDC(map):
    servo = {}
    for k, v in map.items():
        GPIO.setup(k, GPIO.OUT)
        servo[k] = GPIO.PWM(k, 50)
        servo[k].start(0.0)
    time.sleep(0.1)
    for k, v in map.items():
        servo[k].ChangeDutyCycle(v)
    time.sleep(0.5)
    for k, v in servo.items():
        v.stop()

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

# centering(18, 7, 10, 4)
# centering(12, 6, 7, 5)

changeDC({18: 7, 12: 6})
changeDC({18: 10, 12: 7})
changeDC({18: 4, 12: 5})
changeDC({18: 7, 12: 6})

GPIO.cleanup()
sys.exit(0)
