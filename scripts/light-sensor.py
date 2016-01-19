#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import signal
import sys

GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.IN)

def exit_handler(signal, frame):
    print "Exiting..."
    GPIO.cleanup()
    print "Bye!"
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler)

while(1):
    print GPIO.input(4)
    time.sleep(1)
