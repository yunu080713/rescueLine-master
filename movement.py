#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
# from pixycamev3.pixy2 import Pixy2



ev3 = EV3Brick()
ev3.speaker.beep()

# ==================================== Device setting ==================================== #

# pixy2 = Pixy2(port=1, i2c_address=0x54)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
l_color = ColorSensor(Port.S1)  # left color sensor
r_color = ColorSensor(Port.S2)  # right color sensor


# run() 함수를 이용해서 로봇 강제구동, steering=0: 직진, 100 or -100 포인트턴
def move(steering, speed, value, stop=True):
    global move_set
    left_motor.reset_angle(0); right_motor.reset_angle(0)
    if steering == 0:
        move_value = (360/(move_set['wheel_diameter']*3.14159265))*value
        moving = 0
    elif steering != 0:
        move_value = ((360/(move_set['wheel_diameter']*3.14159265))/
                    (360/((-0.0202*abs(steering)+3.0108)*move_set['axle_track']*3.14159265)))*value
        if -100 > steering:
            st_B = -speed
            st_C = speed
            moving = -1
        elif -100 <= steering < 0:
            st_B = ((speed*2)/100)*steering+speed
            st_C = speed
            moving = -1
        elif 0 < steering <= 100:
            st_B = speed
            st_C = -((speed*2)/100)*steering+speed
            moving = 1
        elif 100 < steering:
            st_B = speed
            TURN = -speed
            moving = 1
    if moving == 0:
        left_motor.run(speed+move_set['weight'])
        right_motor.run(speed-move_set['weight'])
        while (abs(left_motor.angle())+abs(right_motor.angle()))/2 <= move_value:
            pass
    elif moving == -1:
        left_motor.run(st_B+move_set['weight'])
        right_motor.run(st_C-move_set['weight'])
        while right_motor.angle() <= move_value:
            pass
    elif moving == 1:
        left_motor.run(st_B+move_set['weight'])
        right_motor.run(st_C-move_set['weight'])
        while left_motor.angle() <= move_value:
            pass
    if stop:
        left_motor.brake()
        right_motor.brake()

# move() 함수로 전후진 거리 측정(mm), 회전(deg)
def move_measure(steering, value):
    global move_set
    if steering == 0:
        move_value = (360/(move_set['wheel_diameter']*3.14159265))*value
        moving = 0
    else:
        move_value = ((360/(move_set['wheel_diameter']*3.14159265))/
                    (360/((-0.0202*abs(steering)+3.0108)*move_set['axle_track']*3.14159265)))*value
        if steering > 0:
            moving = 1
        else:
            moving = -1
    if moving == 0:
        result = not (abs(left_motor.angle())+abs(right_motor.angle()))/2 >= move_value
    elif moving == 1:
        result = not abs(left_motor.angle()) >= move_value
    else:
        result = not abs(right_motor.angle()) >= move_value
    return result

# ==================================== Preferences ==================================== #


# weight: 우측으로 휘면 -값, 좌측으로 휘면 +값, wheel_diameter: 바퀴지름, axle_track: 바퀴간 거리
move_set = {'weight': -1., 'wheel_diameter': 40.25, 'axle_track': 198}

# ==================================== Main Loop ==================================== #

move(100, 500, 90, stop=True)
wait(1000)
