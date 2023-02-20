#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
ev3.speaker.beep()

l_color = ColorSensor(Port.S1)
r_color = ColorSensor(Port.S2)

def max_most(a):
    a_dict = {}
    for i in a:
        if a_dict.get(i) is None:
            a_dict[i] = 1
        else:
            a_dict[i] += 1
    most = max(a_dict.values())
    for key, value in a_dict.items():
        if value == most:
            max_key = key
    return max_key

def colorSet(rgb, color_dict, lr):
    if colorLine("silver",rgb) > color_dict[(lr+"_silver")]: line = "silver"
    elif colorLine("red", rgb) > color_dict[(lr+"_red")]: line = "red"
    elif colorLine("green", rgb) > color_dict[(lr+"_green")]: line = "green"
    elif colorLine("black", rgb) < color_dict[(lr+"_black")]: line = "black"
    elif colorLine("white", rgb) > color_dict[(lr+"_white")]: line = "white"
    else: line = "none" 
    return line

def rgb_view():
    while True:
        l_rgb_list = []
        r_rgb_list = []
        ev3.screen.clear()
        ev3.screen.draw_text(0, 20, "Press the center")
        ev3.screen.draw_text(0, 50, "button to start,")
        ev3.screen.draw_text(0, 80, "release to end.")
        while not Button.CENTER in ev3.buttons.pressed():
            pass
        ev3.screen.clear()
        ev3.screen.draw_text(0, 50, "Start scanning...")
        while Button.CENTER in ev3.buttons.pressed():
            l_rgb_list.append(l_color.rgb())
            r_rgb_list.append(r_color.rgb())

        r_l = []; g_l = []; b_l = []
        d_l = 0; e_l = 0; f_l = 0
        for i in l_rgb_list:
            d_l, e_l, f_l = i
            r_l.append(d_l); g_l.append(e_l); b_l.append(f_l)
        l_rgb = (max_most(r_l), max_most(g_l), max_most(b_l))

        r_r = []; g_r = []; b_r = []
        d_r = 0; e_r = 0; f_r = 0
        for j in r_rgb_list:
            d_r, e_r, f_r = j
            r_r.append(d_r); g_r.append(e_r); b_r.append(f_r)
        r_rgb = (max_most(r_r), max_most(g_r), max_most(b_r))

        l_g = l_rgb[1]
        r_g = r_rgb[1]
        average = (l_g + r_g)/2
        l = average / l_g
        r = average / r_g
        L = round(l, 2)
        R = round(r, 2)
       
        ev3.screen.clear()        
        ev3.screen.print('L:', l_rgb)
        ev3.screen.print('R:', r_rgb)
        ev3.screen.print('L:', L, 'R:', R)
        ev3.screen.draw_text(0, 75, "Press the up")
        ev3.screen.draw_text(0, 90, "button to return")
        ev3.screen.draw_text(0, 105, "to the beginning.")
        while not Button.UP in ev3.buttons.pressed():
            pass

rgb_view()
