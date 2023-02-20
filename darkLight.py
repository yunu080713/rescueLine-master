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

l_color = ColorSensor(Port.S1)  # left color sensor
r_color = ColorSensor(Port.S2)  # right color sensor

rgb_data = {
    'l_white': (30, 30, 66),
    'r_white': (29, 24, 73),
    'l_black': (3, 3, 7),
    'r_black': (8, 10, 8),
    'l_green': (7, 17, 9), 
    'r_green': (7, 15, 11),
    'l_silver': (100, 100, 100), 
    'r_silver': (100, 100, 100),
    'l_red': (26, 3, 5), 
    'r_red': (26, 2, 6)
}


while True:
    while not Button.CENTER in ev3.buttons.pressed():
        pass
    while Button.CENTER in ev3.buttons.pressed():
        pass
    lrgb = l_color.rgb()
    rrgb = r_color.rgb()
    left = "none"
    right = 'none'

    if rgb_data['l_white'][0] - 2 < lrgb[0]:
        left = 'light'
    elif rgb_data['l_white'][0] -11 < lrgb[0]:
        left = 'dark'
    if rgb_data['r_white'][0] - 2 < rrgb[0]:
        right = 'light'
    elif rgb_data['r_white'][0] -11 < rrgb[0]:
        right = 'dark'
    print('L:{}, rgb:{} ,R:{}, rgb:{}'.format(left, right, lrgb, rrgb))
