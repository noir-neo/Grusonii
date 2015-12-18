#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import signal
import sys
import picamera
import io
import cv2
import numpy as np


print "grusonii is during startup..."

cascade_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml"

servo = None
dcX = 0.0
dcY = 0.0
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

with picamera.PiCamera() as camera:

    DURATION = 1.0
    IMAGE_W = 640
    IMAGE_H = 480
    images = [None] * 3
    hist_32 = np.zeros((IMAGE_H, IMAGE_W), np.float32)

    camera.resolution = (IMAGE_W, IMAGE_H)
    camera.start_preview()
    time.sleep(2)
    stream = io.BytesIO()

    has_previous_diff = False


    def exit_handler(signal, frame):
        print "Exiting..."
        camera.stop_preview()
        if servo:
            servo.stop()
        lookat(7.5, 7)
        GPIO.cleanup()
        print "Bye!"
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_handler)


    def lookat(angleX, angleY):
        global dcX, dcY

        if dcY != angleY:
            dcY = angleY
            if dcY < 4:
                dcY = 4
            if dcY > 10:
                dcY = 10

            changeDC(18, dcY)
            lookat(angleX, dcY)

        elif dcX != angleX:
            dcX = angleX
            if dcX < 4.5:
                dcX = 4.5
            if dcX > 7.5:
                dcX = 7.5

            changeDC(12, dcX)

        reset_image()


    def rotate(angleX, angleY):
        global dcX, dcY

        lookat(dcX + angleX, dcY + angleY)


    def changeDC(gpionum, to):
        servo = GPIO.PWM(gpionum, 50)
        servo.start(0.0)
        time.sleep(0.1)
        servo.ChangeDutyCycle(to)
        time.sleep(0.5)
        servo.stop()
        servo = None


    def get_diff():
        if any([v == None for v in images]):
            return False

        d1 = cv2.absdiff(images[2], images[1])
        d2 = cv2.absdiff(images[1], images[0])
        diff = cv2.bitwise_and(d1, d2)
        retval, mask = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

        return mask


    def get_motion_angle(diff):
        times_s = time.clock()
        cv2.updateMotionHistory(diff, hist_32, times_s, DURATION)
        hist_8, direction = cv2.calcMotionGradient(hist_32,
            0.25,
            0.05,
            apertureSize=5)
        angle = cv2.calcGlobalOrientation(
            direction, hist_8, hist_32, times_s, DURATION)

        return angle


    def find_face(image_gray):
        stream.seek(0)

        stream.seek(0)
        stream.truncate()

        cascade = cv2.CascadeClassifier(cascade_path)

        facerect = cascade.detectMultiScale(image_gray,
            scaleFactor=1.1,
            minNeighbors=1,
            minSize=(1, 1))

        if len(facerect) > 0:
            for rect in facerect:
                cv2.rectangle(image_gray,
                    tuple(rect[0:2]),
                    tuple(rect[0:2]+rect[2:4]),
                    (255,255,255),
                    thickness=2)
            cv2.imwrite("detected.jpg", image_gray)

        return facerect


    def read_image():
        stream.seek(0)

        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(data, 1)
        image_gray = cv2.cvtColor(image, cv2.cv.CV_BGR2GRAY)

        images[0] = images[1]
        images[1] = images[2]
        images[2] = image_gray

        stream.seek(0)
        stream.truncate()


    def reset_image():
        images = [None] * 3



    def action():
        global has_previous_diff

        if has_previous_diff:
            has_previous_diff = False
            face = find_face(images[2])
            if len(face) > 0:
                point = face[0][0:2]+face[0][2:4]
                print point
                px = (point[0] - (IMAGE_W / 2)) / 100
                py = (point[1] - (IMAGE_H / 2)) / 100
                rotate(px, py)

            return

        diff = get_diff()
        if diff is False:
            return

        diff_rate = cv2.countNonZero(diff)
        # print "diff_rate: {0}".format(diff_rate)
        if diff_rate > IMAGE_W * IMAGE_H * 0.01:
            # has_previous_diff = True

            angle_deg = get_motion_angle(diff)
            print "angle: {0}".format(angle_deg)
            if angle_deg and angle_deg != 0.0:
                angle_rad = np.radians(angle_deg)
                px = np.sin(angle_rad)/2
                py = np.cos(angle_rad)
                print "x: {0}, y: {1}".format(px, py)
                rotate(px, py)

                return

            # rotate(np.random.rand()*2-1, np.random.rand()*4-2)
            # time.sleep(0.5)


    def init():
        lookat(6, 7)
        print "Hi!"


    def main():
        for foo in camera.capture_continuous(stream, "jpeg", use_video_port=True):
            read_image()
            action()


    init()
    main()
