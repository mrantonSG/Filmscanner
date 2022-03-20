#!/usr/bin/env python3.7

import datetime
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import os.path
import cv2
import random
import glob
import subprocess
import sys


DIR = 27   # Direction GPIO Pin
STEP = 22  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
STEPON = 0  # pin to turn on/off the stepper

ycal = 50  # calibrate camera frame y position 0=center of blob
xcal = 0
ysize = 494  # needs to be adjusted to fit the picture
xsize = 1310
xstart = 290  # x startpoint

BtnLeft = 13    # buttons
BtnRight = 19
BtnStart = 16
BtnStop = 26
BtnRew = 20

Photoint = 21  # photointeruptor
ledon = 12  # pin for LED
pinForward = 6  # motor pin (spool)
pinBackward = 5

step_count = 100  # steps per frame (S8)
delay = .001  # delay inbetween steps

midy = 240  # blob (S8)
tolerance = 10  # pixel tolerance
uptol = midy + tolerance
downtol = midy - tolerance
tolstep = tolerance // 2  # defines how many steps are done for correction
steps = 0

scanDir = '/home/pi/scanframes/'
cropPath = '/home/pi/cropframes/'

stepMinus = 0  # counter for stepper corrections
stepPlus = 0
r = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(STEPON, GPIO.OUT)
GPIO.setup(ledon, GPIO.OUT)
GPIO.setup(pinForward, GPIO.OUT)
GPIO.setup(pinBackward, GPIO.OUT)
GPIO.setup(BtnRight, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BtnLeft, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BtnStart, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BtnStop, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BtnRew, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(Photoint, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup((18, 15, 14), GPIO.OUT)

pwm = GPIO.PWM(6, 40)  # set PWM channel, hz


def set_camera():
    global camera
    camera = PiCamera()
    camera.sensor_mode = 6
    camera.rotation = 180
    camera.resolution = (2048, 1536)  # (1920,1440)#(1640,1232)#(3280,2464)
    camera.awb_mode = 'off'
    camera.awb_gains = [1.8, 1.1]
    sleep(2)


def motorStart():  # for spoolmotor
    if GPIO.input(Photoint):
        pwm.start(10)
        #GPIO.output(pinBackward, GPIO.LOW)

    else:
        pwm.ChangeDutyCycle(0)
        #GPIO.output(pinBackward, GPIO.LOW)


def motorStop():
    pwm.ChangeDutyCycle(0)


def spool():
    if r == 1:
        GPIO.output(pinForward, GPIO.HIGH)
        GPIO.output(pinBackward, GPIO.LOW)
    else:
        GPIO.output(pinForward, GPIO.LOW)
        GPIO.output(pinBackward, GPIO.LOW)


def stepCW(steps):
    for x in range(steps):
        GPIO.output(DIR, CW)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)


def stepCCW(steps):
    for x in range(steps):
        GPIO.output(DIR, CCW)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)


def stop_scanner():
    motorStop()
    GPIO.output(ledon, GPIO.LOW)
    GPIO.output(STEPON, GPIO.LOW)
    camera.stop_preview()


def takePicture():

    global image, oimage
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    oimage = image
    image = cv2.resize(image, (640, 480))


def find_blob(area_size):

    global image, cY, M, area

    #image = cv2.resize(image, (640, 480))
    image = image[0:480, 40:80]  # [80:400, 40:80]
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_image, 200, 255, 0)
    im, contours, hierarchy = cv2.findContours(thresh, 1, 2)

    # cv2.imshow("Output",thresh)
    # cv2.waitKey(0)

    for l in range(10):  # if more contours are found, take the one that's area is >2000
        cnt = contours[l]
        area = cv2.contourArea(cnt)
        print(area)
        if area > area_size:
            break
    M = cv2.moments(cnt)

    try:
        #cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    except ZeroDivisionError:
        #cX = 22
        cY = 240
    print(cY)


def endocv():
    for k in range(8):
        cv2.destroyAllWindows()
        cv2.waitKey(1)


def calPic():

    global image, randompic

    img = cv2.imread(scanDir + randompic)
    print(randompic)
    image = cv2.resize(img, (640, 480))

    find_blob(2000)

    LMP = int(cY * 3.2)+ycal  # x size of scanned image 2084 / 640
    cv2.rectangle(img, (xstart+xcal, LMP-ysize), (xstart+xsize+xcal, LMP+ysize), (0, 255, 0), 50)
    cv2.imshow('Cal-Crop', img)
    cv2.waitKey(50)


def cropPic():

    global image

    img = cv2.imread(n)
    image = cv2.resize(img, (640, 480))

    find_blob(2000)

    LMP = int(cY * 3.2)+ycal
    img = img[LMP-ysize:LMP+ysize, xstart+xcal:xstart+xsize+xcal]

    cv2.imwrite(os.path.join(cropPath, n), img)
    cv2.waitKey(25)
    cv2.imshow('Cal-Crop', img)
    cv2.waitKey(50)


def start_scanner():
    global max_pic_num
    max_pic_num = len([f for f in os.listdir(scanDir)if os.path.isfile(
        os.path.join(scanDir, f))])  # - number of files in directory
    set_camera()
    GPIO.output((18, 15, 14), (1, 1, 0))
    GPIO.output(ledon, GPIO.HIGH)  # turn on LED
    GPIO.output(STEPON, GPIO.HIGH)
    sleep(2)
    camera.start_preview()
    sleep(1)


def cal_crop():

    global randompic, ycal, xcal, n, r

    xy = 1

    cv2.namedWindow('Cal-Crop', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Cal-Crop', 640, 480)
    randompic = random.choice(os.listdir('/home/pi/scanframes/'))
    calPic()

    while True:

        spool()

        if GPIO.input(BtnStart) == GPIO.LOW:
            randompic = random.choice(os.listdir('/home/pi/scanframes/'))
            calPic()
            sleep(2)
            if GPIO.input(BtnStart) == GPIO.LOW and GPIO.input(BtnRight) == GPIO.LOW:
                os.chdir(scanDir)
                for n in sorted(glob.glob('*.jpg')):
                    cropPic()
                    print(n)
                    if GPIO.input(BtnStop) == GPIO.LOW:
                        endocv()
                        sys.exit(0)
                cv2.waitKey(1)
                endocv()
                os.chdir('/home/pi/cropframes/')
                subprocess.check_output(
                    'ffmpeg -r 18 -f image2 -s 1920x1080  -pattern_type glob -i "*.jpg" -vcodec libx264 -preset ultrafast -crf 10  -vf format=yuv420p film.mp4', shell=True)

                print("done")
                endocv()
                sys.exit(0)

        if GPIO.input(BtnLeft) == GPIO.LOW:
            if xy == 1:
                ycal += 10
                calPic()
            else:
                xcal += 10
                calPic()

        if GPIO.input(BtnRight) == GPIO.LOW:
            if xy == 1:
                ycal -= 10
                calPic()
            else:
                xcal -= 10
                calPic()

        if GPIO.input(BtnStop) == GPIO.LOW:
            if xy == 1:
                xy += 1
                sleep(1)
            else:
                xy -= 1
                sleep(1)
            if GPIO.input(BtnStop) == GPIO.LOW:
                print("push2")
                endocv()
                sys.exit(0)

        if GPIO.input(BtnRew) == GPIO.LOW:
            if r == 0:
                r += 1
            else:
                r -= 1


if __name__ == '__main__':

    start_scanner()

    try:

        while True:

            spool()
            motorStart()

            if GPIO.input(BtnRight) == GPIO.LOW:  # step to adjust frame
                stepCW(tolstep)
                sleep(0.05)

            if GPIO.input(BtnLeft) == GPIO.LOW:  # step to adjust frame
                stepCCW(tolstep)
                sleep(0.05)

            if GPIO.input(BtnRew) == GPIO.LOW:  # rewind

                if r == 0:
                    r += 1
                else:
                    r -= 1

            if GPIO.input(BtnStart) == GPIO.LOW:  # start recording

                while GPIO.input(BtnStop):
                    motorStart()
                    takePicture()
                    find_blob(2000)

                    if area > 8000:  # end of film
                        stop_scanner()
                        cal_crop()

                    if cY > uptol:
                        stepCW(tolstep)
                        stepMinus += 1
                        print(stepMinus)

                    if cY < downtol:
                        stepCCW(tolstep)
                        stepPlus += 1
                        print(stepPlus)

                    if cY <= uptol and cY >= downtol:

                        camera.capture('/home/pi/scanframes/scan' + ' - ' +
                                       format(max_pic_num, '06') + '.jpg', use_video_port=True)

                        stepCW(step_count)
                        stepMinus = 0
                        stepPlus = 0
                        max_pic_num += 1

            if GPIO.input(BtnStop) == GPIO.LOW:
                print("push1")
                sleep(5)
                if GPIO.input(BtnStop) == GPIO.LOW:
                    print("push2")
                    stop_scanner()
                    cal_crop()

    except KeyboardInterrupt:
        stop_scanner()
        GPIO.cleanup()
        sys.exit(0)
