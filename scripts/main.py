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

servo_gpionum = {'x': 12, 'y': 18}
servo = {}
dc = {'x': 6.0, 'y': 7.0}

light_gpionum = 4
sleep_flag_count = 0


with picamera.PiCamera() as camera:

    DURATION = 1.0
    IMAGE_W = 640
    IMAGE_H = 480
    images = [None] * 3
    hist_32 = np.zeros((IMAGE_H, IMAGE_W), np.float32)

    camera.resolution = (IMAGE_W, IMAGE_H)
    stream = io.BytesIO()

    has_previous_diff = False

    def start_camera():
        camera.start_preview()
        time.sleep(2)


    def stop_camera():
        camera.stop_preview()


    def exit_handler(signal, frame):
        print "Exiting..."
        stop_camera()
        change_dc({'x': 7.5, 'y': 7})
        GPIO.cleanup()
        print "Bye!"
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_handler)


    def lookat(angleX, angleY):
        global dc

        if dc['y'] == angleY and dc['x'] == angleX:
            return

        if dc['y'] != angleY:
            dc['y'] = angleY
            if dc['y'] < 4:
                dc['y'] = 4
            if dc['y'] > 10:
                dc['y'] = 10

        if dc['x'] != angleX:
            dc['x'] = angleX
            if dc['x'] < 4.5:
                dc['x'] = 4.5
            if dc['x'] > 7.5:
                dc['x'] = 7.5

        reset_image()
        change_dc(dc)


    def rotate(angleX, angleY):
        global dc

        lookat(dc['x'] + angleX, dc['y'] + angleY)


    def change_dc(map):
        start_servo()

        for k, v in map.items():
            servo[k].ChangeDutyCycle(v)
        time.sleep(0.5)

        stop_servo()


    def start_servo():
        for k, v in servo_gpionum.items():
            servo[k] = GPIO.PWM(v, 50)
            servo[k].start(0.0)

        time.sleep(0.1)


    def stop_servo():
        for k, v in servo.items():
            v.stop()


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

    def check_awake():
        if should_sleep():
            change_dc({'x': 7.5, 'y': 7})
            stop_camera()

            while should_sleep():
                time.sleep(5)

            start_camera()
            change_dc({'x': 6, 'y': 7})


    def should_sleep():
        global sleep_flag_count

        if is_bright():
            if sleep_flag_count > 0:
                sleep_flag_count -= 1

        else:
            if sleep_flag_count < 100:
                sleep_flag_count += 1

        return sleep_flag_count > 95


    def is_bright():
        # dark -> 1, bright -> 0
        return GPIO.input(4) < 1


    def init():
        start_camera()

        GPIO.setmode(GPIO.BCM)
        for k, v in servo_gpionum.items():
            GPIO.setup(v, GPIO.OUT)

        GPIO.setup(light_gpionum, GPIO.IN)

        change_dc(dc)
        print "Hi!"


    def main():
        for foo in camera.capture_continuous(stream, "jpeg", use_video_port=True):

            check_awake()

            read_image()
            action()


    init()
    main()
