#!/usr/bin/env python3.7

# Version 2.0.1
# March 21 2022


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

# Stepper motor settings
DIR = 27   # Direction GPIO Pin
STEP = 22  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
STEPON = 0  # pin to turn on/off the stepper

# crop frame settings
ycal = 500  # calibrate camera frame y position 0=center of blob
xcal = 10
ysize = 494  # needs to be adjusted to fit the picture
xsize = 1310
xstart = 300  # x startpoint

# buttons
btn_left = 13
btn_right = 19
btn_start = 16
btn_stop = 26
btn_rewind = 20

photoint = 21  # photointeruptor
ledon = 12  # pin for LED
pin_forward = 6  # motor pin (spool)
pin_backward = 5

step_count = 80  # steps per frame (R8)
delay = .001  # delay inbetween steps

midy = 100  # blob (S8)
tolerance = 10  # pixel tolerance
uptol = midy + tolerance
downtol = midy - tolerance
tolstep = tolerance // 2  # defines how many steps are done for correction
steps = 0

scan_dir = '/home/pi/scanframes/'
crop_path = '/home/pi/cropframes/'

step_minus = 0  # counter for stepper corrections
step_plus = 0
rewind = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(STEPON, GPIO.OUT)
GPIO.setup(ledon, GPIO.OUT)
GPIO.setup(pin_forward, GPIO.OUT)
GPIO.setup(pin_backward, GPIO.OUT)
GPIO.setup(btn_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn_start, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn_stop, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(btn_rewind, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(photoint, GPIO.IN, pull_up_down=GPIO.PUD_UP)
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


def motor_start():  # for spoolmotor
    if GPIO.input(photoint):
        pwm.start(10)
        #GPIO.output(pin_backward, GPIO.LOW)
    else:
        pwm.ChangeDutyCycle(0)
        #GPIO.output(pin_backward, GPIO.LOW)


def motor_stop():
    pwm.ChangeDutyCycle(0)


def spool():
    if rewind == 1:
        GPIO.output(pin_forward, GPIO.HIGH)
        GPIO.output(pin_backward, GPIO.LOW)
    else:
        GPIO.output(pin_forward, GPIO.LOW)
        GPIO.output(pin_backward, GPIO.LOW)


def step_cw(steps):
    for x in range(steps):
        GPIO.output(DIR, CW)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)


def step_ccw(steps):
    for x in range(steps):
        GPIO.output(DIR, CCW)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)


def stop_scanner():
    motor_stop()
    GPIO.output(ledon, GPIO.LOW)
    GPIO.output(STEPON, GPIO.LOW)
    camera.stop_preview()


def take_picture():
    global image, oimage
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    oimage = image
    image = cv2.resize(image, (640, 480))

# area size has to be set to identify the sprocket hole blob
# if the sprocket hole area is around 2500, then 2000 should be a safe choice
# the area size will trigger the exit from the loop


def find_blob(area_size):
    global image, cY, M, area
    image = image[240:480, 40:80]
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_image, 200, 255, 0)
    im, contours, hierarchy = cv2.findContours(thresh, 1, 2)

    # to see the part of the image with the sprocket hole -
    # uncomment the next 2 lines
    # waitkey(0) - picture is displayed until a key is pressed
    # cv2.imshow("Output",thresh)
    # cv2.waitKey(0)

    for l in range(10):
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


def cal_pic():
    global image, randompic
    img = cv2.imread(scan_dir + randompic)
    print(randompic)
    image = cv2.resize(img, (640, 480))
    find_blob(400)
    LMP = int(cY * 3.2)+ycal
    cv2.rectangle(img, (xstart+xcal, LMP-ysize),
                  (xstart+xsize+xcal, LMP+ysize), (0, 255, 0), 50)
    cv2.imshow('Cal-Crop', img)
    cv2.waitKey(50)


def crop_pic():
    global image
    img = cv2.imread(n)
    image = cv2.resize(img, (640, 480))
    find_blob(400)
    LMP = int(cY * 3.2)+ycal
    img = img[LMP-ysize:LMP+ysize, xstart+xcal:xstart+xsize+xcal]
    cv2.imwrite(os.path.join(crop_path, n), img)
    cv2.waitKey(25)
    cv2.imshow('Cal-Crop', img)
    cv2.waitKey(50)


def start_scanner():
    global max_pic_num
    max_pic_num = len([f for f in os.listdir(scan_dir)if os.path.isfile(
        os.path.join(scan_dir, f))])  # - number of files in directory
    set_camera()
    GPIO.output((18, 15, 14), (1, 1, 0))
    GPIO.output(ledon, GPIO.HIGH)  # turn on LED
    GPIO.output(STEPON, GPIO.HIGH)
    sleep(2)
    camera.start_preview()
    sleep(1)


def cal_crop():
    global randompic, ycal, xcal, n, rewind
    xy = 1
    cv2.namedWindow('Cal-Crop', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Cal-Crop', 640, 480)
    randompic = random.choice(os.listdir('/home/pi/scanframes/'))
    cal_pic()

    while True:

        spool()

        if GPIO.input(btn_start) == GPIO.LOW:
            randompic = random.choice(os.listdir('/home/pi/scanframes/'))
            cal_pic()
            sleep(2)
            if GPIO.input(btn_start) == GPIO.LOW and GPIO.input(btn_right) == GPIO.LOW:
                os.chdir(scan_dir)
                for n in sorted(glob.glob('*.jpg')):
                    crop_pic()
                    print(n)
                    if GPIO.input(btn_stop) == GPIO.LOW:
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

        if GPIO.input(btn_left) == GPIO.LOW:
            if xy == 1:
                ycal += 10
                cal_pic()
            else:
                xcal += 10
                cal_pic()

        if GPIO.input(btn_right) == GPIO.LOW:
            if xy == 1:
                ycal -= 10
                cal_pic()
            else:
                xcal -= 10
                cal_pic()

        if GPIO.input(btn_stop) == GPIO.LOW:
            if xy == 1:
                xy += 1
                sleep(1)
            else:
                xy -= 1
                sleep(1)
            if GPIO.input(btn_stop) == GPIO.LOW:
                print("push2")
                endocv()
                sys.exit(0)

        if GPIO.input(btn_rewind) == GPIO.LOW:
            if rewind == 0:
                rewind += 1
            else:
                rewind -= 1


if __name__ == '__main__':

    start_scanner()

    try:

        while True:

            spool()
            motor_start()

            if GPIO.input(btn_right) == GPIO.LOW:  # step to adjust frame
                step_cw(tolstep)
                sleep(0.05)

            if GPIO.input(btn_left) == GPIO.LOW:  # step to adjust frame
                step_ccw(tolstep)
                sleep(0.05)

            if GPIO.input(btn_rewind) == GPIO.LOW:  # rewind

                if rewind == 0:
                    rewind += 1
                else:
                    rewind -= 1

            if GPIO.input(btn_start) == GPIO.LOW:  # start recording

                while GPIO.input(btn_stop):
                    motor_start()
                    take_picture()
                    find_blob(400)

                    if area > 8000:  # end of film
                        stop_scanner()
                        cal_crop()

                    if cY > uptol:
                        step_cw(tolstep)
                        step_minus += 1
                        print(step_minus)

                    if cY < downtol:
                        step_ccw(tolstep)
                        step_plus += 1
                        print(step_plus)

                    if cY <= uptol and cY >= downtol:

                        camera.capture('/home/pi/scanframes/scan' + ' - '
                                       + format(max_pic_num, '06') + '.jpg', use_video_port=True)

                        step_cw(step_count)
                        step_minus = 0
                        step_plus = 0
                        max_pic_num += 1

            if GPIO.input(btn_stop) == GPIO.LOW:
                print("push1")
                sleep(5)
                if GPIO.input(btn_stop) == GPIO.LOW:
                    print("push2")
                    stop_scanner()
                    cal_crop()

    except KeyboardInterrupt:
        stop_scanner()
        GPIO.cleanup()
        sys.exit(0)
