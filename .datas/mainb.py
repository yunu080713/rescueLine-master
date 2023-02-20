#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

l_color = ColorSensor(Port.S1)
r_color = ColorSensor(Port.S2)
# pixy2 = Pixy2()

# ===============datas==================
rgb_data = {
    "l_black": (6,3,11),
    "r_black": (5,3,8),
    "l_silver": (52,59,100),
    "r_silver": (39,20,100),
    "l_red": (20,1,4),
    "r_red": (22,2,5),
    "l_green": (3,6,4),
    "r_green": (6,10,5),
    "l_white": (17,14,43),
    "r_white": (18,16,41)
}

# weight: 우측으로 휘면 -값, 좌측으로 휘면 +값, wheel_diameter: 바퀴지름, axle_track: 바퀴간 거리
move_set = {'weight': -2, 'wheel_diameter': 39.75, 'axle_track': 168.7}

# ===============functions===================
def colorLine(color, inValue):
    inValue = list(inValue)
    for i in range(3):
        if inValue[i] == 0:
            inValue[i] = 1

    r,g,b = inValue
    rs = r/sum(inValue)*100
    gs = g/sum(inValue)*100
    bs = b/sum(inValue)*100

    if color == 'red':
        value = r-10
    elif color == "green":
        value = gs-r/(r+g)*100-5
    elif color == "silver" or color == "white":
        value = sum(inValue)
    elif color == "black":
        value = g+5 # black < value
    return value

# def line_color(l_line, r_line):
#     return l_line == 'white' and r_line == "white",
#            l_line == "green" or r_line == "green",
#            l_line == "red" or r_line == "red",
#            l_line == "silver" or r_line == "silver"
    
def colorReference(rgbData):
    rgb_data["l_white"][1]
    rgb_data["l_black"][1]
    color_dict = {}
    color_dict["l_silver"] = (colorLine("silver", rgbData["l_silver"]) + colorLine("white", rgbData["l_white"]))/2
    color_dict["r_silver"] = (colorLine("silver", rgbData["r_silver"]) + colorLine("white", rgbData["r_white"]))/2
    color_dict["l_red"] = colorLine("red", rgbData["l_red"])
    color_dict["r_red"] = colorLine("red", rgbData["r_red"])
    color_dict["l_green"] = colorLine("green", rgbData["l_green"])
    color_dict["r_green"] = colorLine("green", rgbData["r_green"])
    color_dict["l_white"] = colorLine("white", rgbData["l_white"]) - 2
    color_dict["r_white"] = colorLine("white", rgbData["r_white"]) - 2
    color_dict["l_black"] = colorLine("black", rgbData["l_black"]) + 3
    color_dict["r_black"] = colorLine("black", rgbData["r_black"]) + 3
    l_threshold = (rgbData["l_black"][1] + rgbData["l_white"][1])/2
    r_threshold = (rgbData["r_white"][1] + rgbData["r_black"][1])/2
    deep_l = rgbData["l_black"][1] + 3
    deep_r = rgbData["r_black"][1] + 3
    return color_dict, l_threshold, r_threshold, deep_l, deep_r

def color_detect(color_dict, tf=False):
    l_rgb = l_color.rgb()
    r_rgb = r_color.rgb()
    leftSensor = l_rgb[1]
    rightSensor = r_rgb[1]
    
    l_line = colorSet(l_rgb, color_dict, "l")
    r_line = colorSet(r_rgb, color_dict, "r")
    if tf:
        return l_line, r_line, leftSensor, rightSensor, l_rgb, r_rgb
    return l_line, r_line, leftSensor, rightSensor

def colorSet(rgb, color_dict, lr):
    if colorLine("silver",rgb) > color_dict[(lr+"_silver")]: line = "silver"
    elif colorLine("red", rgb) > color_dict[(lr+"_red")]: line = "red"
    elif colorLine("green", rgb) > color_dict[(lr+"_green")]: line = "green"
    elif colorLine("black", rgb) < color_dict[(lr+"_black")]: line = "black"
    elif colorLine("white", rgb) > color_dict[(lr+"_white")]: line = "white"
    else: line = "none" 
    return line

# R, G, B로 분석한 값 출력 함수 & Two sensor line tracing - run()사용
def pid_control(l_sensor, r_sensor, speed, kp, ki, kd, min_speed, st_ratio = 0, mode = 'dc'):
    # 전역변수를 지역변수로
    global integral; global last_error; global max_error
    # 라인트레이싱 - PID control
    error = l_sensor - r_sensor
    integral += error
    differential = error - last_error
    # 적분 상수를 0으로 입력할 경우
    if ki == 0:
        max_integral = 0
    else:
        max_integral = (speed - (max_error * kp)) / ki
    if abs(integral) >= max_integral:
        if integral > 0:
            integral = max_integral
        else:
            integral = -max_integral
    steering = (kp * error) + (ki * integral) + (kd * differential)
    # 두개의 센서가 모두 백색에 위치했을 때 전진할 수 있는 값
    if st_ratio > steering > -st_ratio:
        steering = 0
    # 제어량이 작으면 speed up, 제어량이 많으면 speed down, 
    if steering >= 0:
        drive_speed = speed - steering
    else:
        drive_speed = speed + steering
    if drive_speed < min_speed:
        drive_speed = min_speed
    # run() 함수를 사용하기 위한 speed값 구하기: left, right
    left = drive_speed+steering
    if left > speed:
        left = speed
    elif left < -speed:
        left = -speed
    right = drive_speed-steering
    if right > speed:
        right = speed
    elif right < -speed:
        right = -speed
    if mode == 'dc':
        # dc()함수로 구동
        left_motor.dc(left)
        right_motor.dc(right)
    elif mode == 'run':
        # run()함수로 구동
        left_motor.run(left)
        right_motor.run(right)
    last_error = error

# run() 함수를 이용해서 로봇 강제구동, steering=0: 직진
def move(steering, speed, mm, deg, stop=True):
    global move_set
    left_motor.reset_angle(0); right_motor.reset_angle(0)
    if steering == 0:
        move_mm = (360/(move_set['wheel_diameter']*3.14159265))*mm
        moving = 'straight'
    elif steering != 0:
        move_deg = ((360/(move_set['wheel_diameter']*3.14159265))/
                    (360/((-0.0202*abs(steering)+3.0108)*move_set['axle_track']*3.14159265)))*deg
        if -100 > steering:
            st_B = -speed
            st_C = speed
            moving = 'left'
        elif -100 <= steering < 0:
            st_B = ((speed2)/100)*steering+speed
            st_C = speed
            moving = 'left'
        elif 0 < steering <= 100:
            st_B = speed
            st_C = -((speed2)/100)*steering+speed
            moving = 'right'
        elif 100 < steering:
            st_B = speed
            TURN = -speed
            moving = 'right'
    if moving == 'straight':
        left_motor.run(speed+move_set['weight'])
        right_motor.run(speed-move_set['weight'])
        while (left_motor.angle()+right_motor.angle())/2 >= move_mm:
            pass
    elif moving == 'left':
        left_motor.run(st_B+move_set['weight'])
        right_motor.run(st_C-move_set['weight'])
        while right_motor.angle() <= move_deg:
            pass
    elif moving == 'right':
        left_motor.run(st_B+move_set['weight'])
        right_motor.run(st_C-move_set['weight'])
        while left_motor.angle() <= move_deg:
            pass
    if stop:
        left_motor.brake()
        right_motor.brake()

# 라인을 가운데두고 좌우센서 위치를 정렬
def row_align(kp, kd, ratio):   # ratio는 정렬을 멈추는 범위값
    global last_error
    # 라인정렬 - PID control
    while True:
        error = l_color.rgb()[1] - r_color.rgb()[1]
        differential = error - last_error
        # PID control값을 steering_drive()의 steering 파라미터로 사용
        steering = (kp * error) + (kd * differential)
        # 두개의 센서가 라인의 경계위치에 정렬되면 정렬 종료
        if ratio > steering > -ratio:
            break
        # 라인 정렬을 위해서 포인트턴
        left_motor.run(steering)
        right_motor.run(-steering)
        last_error = error
    left_motor.brake()
    right_motor.brake()

# move() 함수로 전후진 거리 측정(mm)
def move_measure(mm):
    global move_set
    move_mm = (360/(move_set['wheel_diameter']*3.14159265))*mm
    result = not (abs(left_motor.angle())+abs(right_motor.angle()))/2 >= move_mm
    return result

def message(text_1, text_2):
    ev3.screen.clear()
    ev3.screen.draw_text(0, 40, text_1)
    ev3.screen.draw_text(50, 60, text_2)


































# ===============preference=================== 

# 라인트레이싱 구동함수 교체 경계값설정
speed_threshold = 100

# 라인트레이싱 설정값 정의 dc: 0~100%, run: deg/s
dc_mode = {'speed':70, 'kp':10, 'ki':0, 'kd':6, 'min_speed':50, 'st_ratio':8, 'mode':'dc'}
run_mode = {'speed':500, 'kp':15, 'ki':0.002, 'kd':30, 'min_speed': 135, 'st_ratio':8,'mode':'run'}

color_dict, threshold_l, threshold_r, deep_l, deep_r = colorReference(rgb_data)
i = 0

#test #0000
while True:
        while not Button.CENTER in ev3.buttons.pressed():
            pass
        while Button.CENTER in ev3.buttons.pressed():
            pass
        a = color_detect(color_dict, True)
        ev3.screen.print(i)
        ev3.screen.print(a, )
        print(i)
        print(a)
        i += 1
# ===============running=================== 

# pid control setup #0001
integral = 0
last_error = 0
max_error = (rgb_data["l_white"][1] - threshold_l) + (rgb_data["r_white"][1] - threshold_r) / 2

data = DataLog("lcolor","rcolor","l","r", name="colordata", timestamp=False) #debug
data.log("left", "right","lrgb","rrgb") #debug

# main loop #0002
start =0
while True:
    message("press CENTER button", "to Start")
    while not Button.CENTER in ev3.buttons.pressed():
        pass
    # first start run short #0003
    if start == 0:
        start = 1
        move(0, 500, 24, 0, stop=False) # 24mm straight
    # down buttons to stop #0004
    message("Press DOWN button", "to STOP")
    # line following data init # 0006
    l_list = []
    r_list = []

    # main program is start and ev3 down buttons is pressed to stop #0005
    while True:
        # # line following data init # 0006
        # l_list = []
        # r_list = []
        while True:
            # color sensor rgb data #0007
            l_line, r_line, l_sensor, r_sensor, lgb, rgb = color_detect(color_dict, True)
            data.log(l_list,r_list,lgb,rgb) #debug
            # save two colorsensor deteceting color data
            l_list.append(l_line)
            r_list.append(r_line)
            if len(l_list) > 10:
                del l_list[0]
                del r_list[0]
            # if pressed down buttons to True, down button is pressed to loop stop #0008
            pressed = Button.DOWN in ev3.buttons.pressed()
            # left or right detect one or two is silver or green or red to true #0009
            # white, green, red, silver = line_color(l_line, r_line)
            white = l_line == 'white' and r_line == "white"
            green = l_line == "green" or r_line == "green"
            red = l_line == "red" or r_line == "red"
            silver = l_line == "silver" and r_line == "silver"
            # line following loop stop - presse down button or green, red, silver #0010
            if pressed or white or green or red or silver:
                left_motor.brake()
                right_motor.brake()
                break
            # motor's speed is under abs value run() else dc() #0011
            if abs(left_motor.speed()) < speed_threshold and abs(right_motor.speed()) < speed_threshold:
                ev3.light.on(Color.RED)
                lm = run_mode
            else:
                ev3.light.on(Color.GREEN)
                lm = dc_mode
            lm = dc_mode
            # pid two sensor line trainsing #0012
            pid_control(l_sensor, r_sensor, lm['speed'], lm['kp'], lm['ki'], lm['kd'], lm['min_speed'], lm['st_ratio'], lm['mode'])
        #user stop to go first #0013
        if pressed:
            break
        print(l_list)
        print(r_list)
        

        # 라인 이탈 및 gap 처리
        if white:
            # 라인트레이싱 중 측정한 데이터의 검은색이 감지된 횟수찾기
            l_num = l_list.count('black')
            r_num = r_list.count('black')
            if l_num > 0 and r_num > 0:
                if l_num > r_num:
                    w_move = 'left'
                elif l_num < r_num:
                    w_move = 'right'
                else:
                    w_move = 'none'
                    ev3.speaker.beep(3000, 1000)
                    wait(5000)
            elif l_num > 0:
                w_move = 'left'
            elif r_num > 0:
                w_move = 'right'
            else:       #  갭의 경우
                w_move = 'gap'
                ev3.speaker.beep(1000, 100)
            if w_move == 'left':
                left_motor.run(-450)
                right_motor.run(450)
                while l_color.rgb()[1] > threshold_l:
                    pass
                row_align(20, 40, 10)
            elif w_move == 'right':
                left_motor.run(450)
                right_motor.run(-450)
                while r_color.rgb()[1] > threshold_r:
                    pass
                row_align(20, 40, 10)
            elif w_move == 'gap':
                move(0, -300, 5, 0, stop=False)
                while l_color.rgb()[1] > deep_l and r_color.rgb()[1] > deep_r and move_measure(40):
                    pass
                left_motor.brake()
                right_motor.brake()
                # 설정거리만큼 후진했는데 검은 선이 없으므로 gap임
                if not move_measure(40):
                    row_align(20, 40, 10)   # gap부분 강제전진하기전에 라인과 로봇의 위치보정
                    move(0, 500, 250, 0, stop=True)     # 250mm만큼 강제전진(gap최대길이:200mm)
                    if l_color.rgb()[1] < threshold_l or r_color.rgb()[1] < threshold_r:    # 라인위에 있음
                        pass
                    else:   # 라인위에 없음
                        pass
                else:   # gap이 아니라 라인 이탈한 경우
                    row_align(20, 40, 10)
