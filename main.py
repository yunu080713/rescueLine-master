#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pixycamev3.pixy2 import Pixy2, Block

ev3 = EV3Brick()
stopwatch = StopWatch()

# ==================================== Device setting ==================================== #

pixy2 = Pixy2(port=4, i2c_address=0x54)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
l_color = ColorSensor(Port.S1)  # left color sensor
r_color = ColorSensor(Port.S2)  # right color sensor
touch = TouchSensor(Port.S3)

arm_motor = Motor(Port.D)
trash_motor = Motor(Port.A)

mode1 = 0
arm_open = False
trash_open = False

ev3.speaker.beep()


# while True:
#     print(ultra_sensor.distance())

# while True:
#     while True:
#         if Button.UP in ev3.buttons.pressed():
#             while Button.UP in ev3.buttons.pressed():
#                 pass
#             mode1 = 1
#             break

#         elif Button.DOWN in ev3.buttons.pressed():
#             while Button.DOWN in ev3.buttons.pressed():
#                 pass
#             mode1 = 2
#             break
#         pass
#     if mode1 == 1:
#         if arm_open:
#             arm_motor.run_until_stalled(-1000)
#             arm_open = False
#         else:
#             arm_motor.run_until_stalled(1000)
#             arm_open = True
#         pass
#     elif mode1 == 2:
#         if trash_open:
#             trash_motor.run_until_stalled(75)
#             trash_open = False
#         else:
#             trash_motor.run_until_stalled(-75)
#             trash_open = True
#         pass
#     print(arm_open, trash_open, mode1)



# ==================================== Functions ==================================== #

# 중복 계산을 피하기위해 계산식을 함수로 만듦
def color_line(color, in_value):     # color: 'green', 'red', silver'
    if in_value.count(0) > 0:   # 입력된튜플값 중 0이 있으면 1로 바꿈
        in_value = list(in_value)
        for z in range(3):
            if in_value[z] == 0:
                in_value[z] = 1
        in_value = tuple(in_value)
    if color == 'silver':
        value = in_value
    elif color == 'green':
        value = ((in_value[1] / sum(in_value)) * 100) - ((in_value[0] / (in_value[0]+in_value[1])) * 100)
    elif color == 'red':
        value = (in_value[0]/sum(in_value)) * 100
    elif color == 'white':
        value = in_value[2]/in_value[1]*100
    # elif color == 'black':
    #     value = in_value[0]+in_value[1]+(in_value[1]/in_value[2])
    return value

# R, G, B로 분석할 기준 값 출력 함수
def color_reference(rgb_data):
    global line_color; global wb_threshold
    color_dict = {}
    # white 합계와 silver 합계의 경계값을 silver 기준, 측정값의 합계가 기준보다 크면 silver
    # color_dict['l_silver_line'] = (color_line('silver', rgb_data['l_silver']) + color_line('white', rgb_data['l_white'])) / 2
    # color_dict['r_silver_line'] = (color_line('silver', rgb_data['r_silver']) + color_line('white', rgb_data['r_white'])) / 2
    color_dict['l_silver_line'] = 100 - sil_threshold
    color_dict['r_silver_line'] = 100 - sil_threshold
    # red의 비중 -10 이상이면 red이다
    color_dict['l_red_line'] = color_line('red', rgb_data['l_red']) - 10
    color_dict['r_red_line'] = color_line('red', rgb_data['r_red']) - 10
    # (green의 비중-(r/(r+g))*100)-5 값이 ??% 이상이면 green이다.
    color_dict['l_green_line'] = color_line('green', rgb_data['l_green']) - 15
    color_dict['r_green_line'] = color_line('green', rgb_data['r_green']) - 15
    # white 기준 -10을 기준, 이것보다 높으면 흰
    color_dict['l_white_line'] = color_line('white', rgb_data['l_white']) - w_threshold
    color_dict['r_white_line'] = color_line('white', rgb_data['r_white']) - w_threshold
    # black의 green 값 + 5를 기준, 이것보다 낮으면 black
    # color_dict['l_black_line'] = color_line('black', rgb_data['l_black']) + lb_threshold
    # color_dict['r_black_line'] = color_line('black', rgb_data['r_black']) + rb_threshold
    # 경계값 구하기
    threshold_l = (rgb_data['l_white'][line_color] + rgb_data['l_black'][line_color]) / 2
    threshold_r = (rgb_data['r_white'][line_color] + rgb_data['r_black'][line_color]) / 2
    deep_l = rgb_data['l_black'][line_color] + lb_threshold
    deep_r = rgb_data['r_black'][line_color] + rb_threshold
    under_l = rgb_data['l_white'][line_color] - w_threshold
    under_r = rgb_data['r_white'][line_color] - w_threshold
    black_l = rgb_data['l_black'][line_color] -10
    black_r = rgb_data["r_black"][line_color] -10
    return color_dict, threshold_l, threshold_r, deep_l, deep_r, under_l, under_r, black_l, black_r

# 왼쪽:1, 오른:2, black:1, green:3, red:4, white:5, silver:7, none:0
def color_detect(color_dict):
    global deep_l; global deep_r; global under_l; global under_r; global line_color
    # l_color, r_color 센서 측정값 분석 - R,G,B 측정값이 추가되면 같이 추가해야 함
    l_rgb = l_color.rgb()
    r_rgb = r_color.rgb()
    print('L', l_rgb)
    print('R', r_rgb)
    l_sensor = l_rgb[line_color]     # RGB값중 green값으로 라인트레이싱
    r_sensor = r_rgb[line_color]     # RGB값중 green값으로 라인트레이싱
    # 각 색 기준값과 현재 left 센서 측정값 비교 - l_line 결과값
    if color_line('silver', l_rgb)[2] > color_dict['l_silver_line'] and sum(l_rgb) > 150:
        l_line = 17
    elif color_line('red', l_rgb) > color_dict['l_red_line']:
        l_line = 14
    elif color_line('green', l_rgb) > color_dict['l_green_line']:
        l_line = 13
    elif color_line('white', l_rgb) > color_dict['l_white_line']:
        l_line = 15
    # elif color_line('black', l_rgb) < color_dict['l_black_line']:
    #     l_line = 11
    # elif l_sensor >= under_l:
    #     l_line = 15
    elif l_sensor <= deep_l:
        l_line = 11
    else:
        l_line = 10
    # 각 색 기준값과 현재 right 센서 측정값 비교 - r_line 결과값
    if color_line('silver', r_rgb)[2] > color_dict['r_silver_line'] and sum(r_rgb) > 150:
        r_line = 27
    elif color_line('red', r_rgb) > color_dict['r_red_line']:
        r_line = 24
    elif color_line('green', r_rgb) > color_dict['r_green_line']:
        r_line = 23
    elif color_line('white', r_rgb) > color_dict['r_white_line']:
        r_line = 25
    # elif color_line('black', r_rgb) < color_dict['r_black_line']:
    #     r_line = 21
    # elif r_sensor >= under_r:
    #     r_line = 25
    elif r_sensor <= deep_r:
        r_line = 21
    else:
        r_line = 20
    print('l', l_line, 'r', r_line)
    return l_line, r_line, l_sensor, r_sensor
# R, G, B로 분석한 값 출력 함수 & Two sensor line tracing - run()사용
def pid_control(l_sensor, r_sensor, speed, kp, ki, kd, min_speed, st_ratio = 0, mode = 'dc'):
    # 전역변수를 지역변수로
    global integral; global last_error; global max_error
    global threshold_l; global threshold_r; global line_color; global black_l; global black_r
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
    # if l_sensor <= black_l or r_sensor <= black_r:
    #     speed = speed if l_sensor <= black_l else -speed
    #     cs = l_color if l_sensor <= black_l else r_color
    #     thre = threshold_r if l_sensor <= black_l else threshold_l
    #     run(left_motor, -speed, mode)
    #     run(right_motor, speed, mode)
    #     while cs.rgb()[line_color] > thre:
    #         pass
    if mode == 'dc':
        # dc()함수로 구동
        left_motor.dc(left)
        right_motor.dc(right)
    elif mode == 'run':
        # run()함수로 구동
        left_motor.run(left)
        right_motor.run(right)
    last_error = error

def run(motor, speed, mode='dc'):
    if mode == 'dc':
        motor.dc(speed)
    elif mode == 'run':
        motor.run(speed)

# run() 함수를 이용해서 로봇 강제구동, steering=0: 직진, 100: right or -100: left 포인트턴
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
        left_motor.hold()
        right_motor.hold()
        wait(300)

def ball_move(steering, speed, value, stop=True):
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
    re = -1
    if moving == 0:
        left_motor.run(speed+move_set['weight'])
        right_motor.run(speed-move_set['weight'])
        re = move_while(moving, move_value)
    elif moving == -1:
        left_motor.run(st_B+move_set['weight'])
        right_motor.run(st_C-move_set['weight'])
        re = move_while(moving, move_value)
    elif moving == 1:
        left_motor.run(st_B+move_set['weight'])
        right_motor.run(st_C-move_set['weight'])
        re = move_while(1, move_value)
    if stop:
        left_motor.hold()
        right_motor.hold()
        wait(300)
    return re

def move_while(mode, value):
    global signatures
    while True:
        l_line, r_line, l_sensor, r_sensor = color_detect(color_dict)
        silver_color = l_line == 17 and r_line == 27
        silver_bool, silver_block = object_detect(signatures.get('silver'), areas = True)
        #
        silver_ball = silver_bool and silver_block.width * silver_block.height > 2500
        red_bool, red_block = object_detect(signatures.get('red'), areas = True)
        #
        red_ball = red_bool and red_block.width * red_block.height > 20000
        #
        black_color = (l_color.rgb()[line_color] < threshold_l or r_color.rgb()[line_color] < threshold_r)
        #
        # angle = ((abs(left_motor.angle())+abs(right_motor.angle()))/2
        #     if mode == 0 else right_motor.angle() if mode == -1 else left_motor.angle()) <= value
        if mode == 0:
            angle = (abs(left_motor.angle())+abs(right_motor.angle()))/2 > value
        elif mode == -1:
            angle = right_motor.angle() > value
        elif mode == 1:
            angle = left_motor.angle() > value
        if touch.pressed():
            move(0, -500, 50)
            arm_move(-1000)
            move(0, -500, 55)
            if touch.pressed():
                break
        if angle or black_color or red_ball or silver_color or silver_ball:
            break
    if black_color:
        re = 3
    elif silver_ball:
        re = 1
    elif red_ball:
        re = 2
    elif angle:
        re = 123
    elif silver_color:
        re = 4
    else:
        re = -1
    return re

    

        
        

def move_value(value):
    move_value = (360/(move_set['wheel_diameter']*3.14159265))*value
    return move_value

# 라인을 가운데두고 좌우센서 위치를 정렬 - green사용
def row_align(kp, kd, ratio):   # ratio는 정렬을 멈추는 범위값
    global last_error
    # 라인정렬 - PD control
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

def message(text_1, text_2):
    ev3.screen.clear()
    ev3.screen.draw_text(0, 40, text_1)
    ev3.screen.draw_text(50, 60, text_2)

def beep(i, sound=1000, deg=50, time=300):
    if i > 0:
        for j in range(i):
            ev3.speaker.beep(sound,deg)
            wait(time)

def move_distance(angle):
    return angle / (360/(move_set['wheel_diameter']*3.14159265))
        
def object_detect(sig, sigmap =255, DISTANCE = 125, areas= False):
    object_bool = False
    nr_blocks, block = pixy2.get_blocks(sigmap, 3)
    blocks = []
    blk_dict = {}
    sig_block = 0
    b = Block()
    b.width = 0
    b.height = 0
    if nr_blocks > 0:
        if areas:
            for e in block:
                if e.sig == sig and e.y_center > DISTANCE:
                    blocks.append(e.width * e.height)
                    blk_dict[e.width * e.height] = e
            if len(blocks) == 0:
                return False, b
            sig_block = blk_dict[max(blocks)]
            return True, sig_block

            
        for e in block:
            if e.sig == sig:
                sig_block = e
                break
        if sig_block == 0:
            return False,0,0
        x = sig_block.x_center
        y = sig_block.y_center
        distance = DISTANCE * 1 / (1 + y)
        print(distance)
        if distance <= 1:
            object_bool = True
    else:
        if areas:
            b = Block()
            b.width = 0
            b.height = 0
            return False, b
        x = 0; y = 0
    if areas:
        return False, b
    else:
        return object_bool, x, y
        

def cam_move(sig, max_block, minsize = 1000, touch_mode=False, sigmap = 255, DISTANCE = 150):
    def limit_speed(speed):
        """ Limit speed in range [-1000,1000] """
        if speed > 1000:
            speed = 1000
        elif speed < -1000:
            speed = -1000
        return speed
    # Defining constants
    X_REF = 158  # X-coordinate of referencepoint
    # Y_REF = 150  # Y-coordinate of referencepoint
    KP = 0.2     # Proportional constant PID-controller
    KI = 0.02    # Integral constant PID-controller
    KD = 0.1     # Derivative constant PID-controller
    GAIN = 10    # Gain for motorspeed
    MAX_X = 220
    MIN_X = 100
    # Initializing PID variables
    integral_x = 0; derivative_x = 0; last_dx = 0
    integral_y = 0; derivative_y = 0; last_dy = 0
    dy = 100
    sig_block = 0
    distance = 10
    width = False
    b_dict = {}

    print(123)

    while True:
        blocks = []
        # Request block
        nr_blocks, block = pixy2.get_blocks(sigmap, max_block)
        # Extract data
        print(pixy2.get_blocks(sigmap, max_block))
        if nr_blocks > 0:
            for e in block:
                if e.sig == sig:
                    blocks.append(e.width * e.height)
                    b_dict[e.width * e.height] = e
            print(len(blocks))
            if len(blocks) == 0:
                print(blocks)
                continue
            sig_block = b_dict[max(blocks)]
            print(sig_block)
            # SIG1 detected, control motors
            x = sig_block.x_center         # object 중앙의 x축 위치
            y = sig_block.y_center         # object 중앙의 y축 위치
            dx = X_REF - x                # Error in reference to X_REF
            integral_x = integral_x + dx  # Calculate integral for PID
            derivative_x = dx - last_dx   # Calculate derivative for PID
            speed_x = KP*dx + KI*integral_x + KD*derivative_x  # Speed X-direction
            dy = DISTANCE - y                # Error in reference to Y_REF
            integral_y = integral_y + dy  # Calculate integral for PID
            derivative_y = dy - last_dy   # Calculate derivative for PID
            speed_y = KP*dy + KI*integral_y + KD*derivative_y  # Speed Y-direction
            # Calculate motorspeed out of speed_x and speed_y
            # Use GAIN otherwise speed will be to slow,
            # but limit in range [-1000,1000]
            lspeed = limit_speed(GAIN*(speed_y - speed_x))
            rspeed = limit_speed(GAIN*(speed_y + speed_x))
            left_motor.run(speed = round(lspeed))
            right_motor.run(speed = round(rspeed))
            distance = DISTANCE * 1 / (1 + y)
            print(distance)
            if touch_mode:
                if touch.pressed():
                    break
            else:
                if distance <= 1 and (MIN_X < block[0].x_center and MAX_X > block[0].x_center):
                    break
            last_dx = dx                  # Set last error for x
            last_dy = dy                  # Set last error for y
    left_motor.stop()
    right_motor.stop()

def arm_move(front):
    if front:
        pass
    else:
        pass

def moving_while(more=False):
    global line;global red
    while True:
        if more:
            break
        object_bool, block = object_detect(2, areas=True, DISTANCE=90)
        if touch.pressed():
            break
        if l_color.rgb()[2] >= 100 and r_color.rgb()[2] >= 100:
            if not measure == 0:
                move(0, 500, move_distance(((abs(left_motor.angle()) + abs(right_motor.angle()))/2)))
            else:
                move(0, -500, move_distance(((abs(left_motor.angle()) + abs(right_motor.angle()))/2)))
            break
        elif l_color.rgb()[line_color] <= threshold_l and r_color.rgb()[line_color] <= threshold_l:
            line=True
            move(0, -500, 150)
            break
        if object_bool:
            if block.width * block.height > 19000:
                move(0, -500, 25)
                move(speed, 500, 90)
                move(0, 500, 300)
                move(100, 500, 1, stop=False)
                while True:
                    object_bool, x, y = object_detect(2, DISTANCE=90)
                    if object_bool:
                        move(100, 500, 25 )
                        break
                move(100, 500, 180)
                move(0, -200, 1, stop=False)
                wait(5000)
                left_motor.hold()
                right_motor.hold()
                move(0, 500, 50)
                move(-100, 500, 90)
                move(0, 500, 1, stop=False)
                while not touch.pressed():
                    pass
                left_motor.hold()
                right_motor.hold()
                move(0, -500, 90)
                move(50, 500, 43)
                move(0, 500, 25)
                red= True
                break

# ==================================== Preferences ==================================== #


# RGB 측정값 입력
rgb_data = {
    'l_white': (27, 24, 53),
    'r_white': (22, 17, 49),
    'l_black': (4, 3, 6),
    'r_black': (2, 1, 4),
    # 'l_black': (3, 2, 5),
    # 'r_black': (5, 3, 9),
    'l_green': (8, 21, 10), 
    'r_green': (7, 16, 9),
    'l_silver': (52, 49, 100), 
    'r_silver': (46, 32, 100),
    'l_red': (16, 2, 6), 
    'r_red': (15, 2, 6)
}
# rgb_data = {
#     'l_white': (35, 31, 67),
#     'r_white': (34, 28, 70),
#     'l_black': (10, 9, 17),
#     'r_black': (12, 7, 17),
#     # 'l_black': (3, 2, 5),
#     # 'r_black': (5, 3, 9),
#     'l_green': (11, 23, 10), 
#     'r_green': (10, 19, 11),
#     'l_silver': (62, 61, 100), 
#     'r_silver': (74, 49, 100),
#     'l_red': (17, 3, 7), 
#     'r_red': (16, 3, 9)
# }
# 라인트레이싱에 사용할 빛 - line_color=0: red, line_color=1: green, line_color=2: blue
line_color = 0

# 백색과 검은색을 구분하는 경계값 - 숫자가 낮으면 분명한 백색 or 검은색
w_threshold = 2
lb_threshold = 8

rb_threshold = 8
sil_threshold = 10

# weight: 우측으로 휘면 -값, 좌측으로 휘면 +값, wheel_diameter: 바퀴지름, axle_track: 바퀴간 거리
move_set = {'weight': -1, 'wheel_diameter': 40.25, 'axle_track': 203}

# 라인트레이싱 구동함수 교체 경계값설정
speed_threshold = 100

# 라인트레이싱 설정값 정의 d: 0~100%, run: deg/s
dc_mode = {'speed':70, 'kp':4, 'ki':0.07798234212, 'kd':3, 'min_speed':50, 'st_ratio':8, 'mode':'dc'}
run_mode = {'speed':500, 'kp':5, 'ki':0, 'kd':0, 'min_speed': 400, 'st_ratio':8,'mode':'run'}

# pixycam의 signature 번호 입력
signatures = {'aid_kit':1, 'red':2, 'green': 3, 'silver':4}

# ==================================== Main Loop ==================================== #

# rgb_data로 색상 측정의 기준값 만듦
color_dict, threshold_l, threshold_r, deep_l, deep_r, under_l, under_r, black_l, black_r = color_reference(rgb_data)
# pid_control() 함수에 사용할 변수
integral = 0
last_error = 0
max_error = ((rgb_data['l_white'][line_color] - threshold_l) + (rgb_data['r_white'][line_color] - threshold_r)) / 2

ball =0
# 팔 테스트 프로그램
# aa = True
# while True:
#     if aa: arm_motor.run_until_stalled(1000)
#     else: arm_motor.run_until_stalled(-1000)
#     print(aa)
#     aa = not aa
# 카메라의 물체 넓이 비교 프로그램
# while True:
#     a = object_detect(2, areas=True, DISTANCE=90)
#     print(a[0])
#     if a[0] == 0:
#         print(a[1])
#     else:
#         print('area', a[1].width * a[1].height > 2500)
#         print('area', a[1].width * a[1].height)

# re = 0
# red = False
# moving = 0
# speed = 100
# line=False
# tf = False
# move(0, 500, 10, stop = False)
# moving_while()
# wait(100)
# if not red:
#     move(0, -500, 25)
# if not line:
#     move(speed, 500, 90)
#     move(0, -500, 10, stop=False)
#     moving_while(more= not move_measure(0, 300))

# left_motor.brake()
# right_motor.brake()
# for i in range(3):
#     if line:
#         if left_motor.angle() < 0:
#             move(100, 500, 180)
#         move(100, 500, 180)
#         move(0, 500, 150, stop=False)
#         while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l:
#             pass
#         left_motor.brake()
#         right_motor.brake()
#         break
#     move(0, 500, 300)
#     ev3.speaker.beep()
    
#     if not red:
#         left_motor.brake()
#         right_motor.brake()
#         wait(100)
#         move(speed, 500, 90)
#         move(0, -500, 1, stop=False)
#         while True:
#             if not move_measure(0, 400):
#                 print(12345)
#                 break
#             if l_color.rgb()[2] >= 100 and r_color.rgb()[2] >= 100:
#                 move(0, 500, move_distance(((abs(left_motor.angle()) + abs(right_motor.angle()))/2)))
#                 break
#             elif l_color.rgb()[line_color] <= threshold_l and r_color.rgb()[line_color] <= threshold_l:
#                 line=True
#                 move(0, -500, 150)
#                 break
#     red=False
    
#     move(0, 500, 1, stop=False)
#     moving_while()
#     if line:
#         continue
#     left_motor.brake()
#     right_motor.brake()

#     move(speed * -1, 500, 90)
    
#     left_motor.brake()
#     right_motor.brake()
#     move(0, -500, 25)
#     speed = speed * -1
#     red=False

# move(-50, 500, 1, stop=False)
# while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(-50, 25):
#     pass
# left_motor.brake()
# right_motor.brake()
# if not move_measure(-50, 25):
#     move(-50, -500, 1, stop=False)
#     while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(-50, 25):
#         pass
#     left_motor.brake()
#     right_motor.brake()
#     move(50, 500, 1, stop=False)
#     while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(50, 25):
#         pass
#     left_motor.brake()
#     right_motor.brake()
# row_align(20, 50, 10)

# while True:
#     pass















aa = False
aaa=False
start = 0
while True:
    if aaa:
        break
    # 초록의 감지 횃수
    green_detect = 0
    white_detect = 0
    aid_kit_detect = 0
    last_w_move = 0
    last_l = 0
    last_r = 0
    w_move = ''
    # EV3 center 버튼을 누를 때까지 대기 - 시작
    message("Press CENTER button", "to START")
    while not Button.CENTER in ev3.buttons.pressed():
        pass
    # 출발 위치가 T자일 경우를 대비해서 최초 1번 0.2초 강제전진
    if start == 0:
        start = 1
        # move(0, 500, 24, stop=False)     # 약 24mm전진
    # EV3 down 버튼을 누를 때까지 대기 - 정지
    message("Press DOWN button", "to STOP")
    # main 프로그램 시작 EV3 다운버튼이 눌리면 종료
    # 라인따라가는 정보저장 초기화
    l_list = []
    r_list = []
    arm_motor.brake()
    arm_motor.run_until_stalled(1000)
    arm_motor.run_until_stalled(-1000)
    arm_motor.run(-10)
    wait(1000)
    while True:
        # 라인트레이싱이 종료되면 다양한 처리를 하고 다시 라인트레이싱으로 복귀
        while True:
            # 컬러센서의 색상 측정 값 및 rgb()함수의 초록값 
            l_line, r_line, l_sensor, r_sensor = color_detect(color_dict)
            # 현재 두개의 컬러센서가 측정하고 있는 색상 값 저장
            l_list.append(l_line)
            r_list.append(r_line)
            if len(l_list) > 15:
                del l_list[0]
                del r_list[0]
            # 다운 버튼을 눌르면 True, down 버튼 클릭하면 루프 종료
            pressed = Button.DOWN in ev3.buttons.pressed()
            # 좌측, 우측 중 하나 또는 두개가 white, green, silver, red이면 True
            white = l_line == 15 and r_line == 25
            green = l_line == 13 or r_line == 23
            silver = l_line == 17 and r_line == 27
            red = l_line == 14 or r_line == 24
            aid_kit, x, y = False, 0, 0#object_detect(1, DISTANCE=130)
            # 선따라가기 루프 종료 - down 버튼 클릭, green, red, silver
            if pressed or white or green or aid_kit or silver or red:
                left_motor.brake()
                right_motor.brake()
                break
            # 모터 speed의 절대값이 경계값보다 낮으면 run(), 아니면 dc()사용
            if abs(left_motor.speed()) < speed_threshold and abs(right_motor.speed()) < speed_threshold:
                ev3.light.on(Color.RED)
                line_mode = run_mode
            else:
                ev3.light.on(Color.GREEN)
                line_mode = dc_mode
            # PID 투센서 라인트레이싱
            pid_control(l_sensor, r_sensor, line_mode['speed'], line_mode['kp'], line_mode['ki'], line_mode['kd'], 
                        line_mode['min_speed'], line_mode['st_ratio'], line_mode['mode'])
        # 사용자 중지로 처음으로 돌아감
        if pressed or red:
            break

        # 라인 이탈 및 gap 처리
        # 라인 이탈 및 gap 처리
        if white:
            if white_detect == 0:
                move(0, 500, 3, stop=True)
            # 라인이탈 경우중 판 틈새를 이탈로 감지하는 것을 방지하기위해 5회다시 라인트레이싱
            if white_detect == 0:
                white_detect += 1
                continue
            # 라인트레이싱 중 측정한 데이터의 검은색이 감지된 횟수찾기
            l_num = l_list.count(11)
            r_num = r_list.count(21)
            l_num_w = l_list.count(15)
            r_num_w = r_list.count(25)
            for i in range(2):
                if i == 0:
                    n=0
                    continue_num_l = 0
                    for e in range(len(l_list)):
                        if l_list[e] == 11:
                            last_l = e
                            n += 1
                        else:
                            n = 0
                        if n > 2: # 연속으로 11이 있는 경우
                            continue_num_l = n
                else:
                    n=0
                    continue_num_r = 0
                    for e in range(len(r_list)):
                        if r_list[e] == 21:
                            last_r = e
                            n += 1
                        else:
                            n = 0
                        if n > 2: # 연속으로 21이 있는 경우
                            continue_num_r = n
            if l_num > 0 and r_num > 0:
                if l_num > r_num:    # L > R 
                    if last_l > last_r:
                        w_move = 'left'
                    elif last_l < last_r:
                        if l_num_w > r_num_w:
                            w_move = 'left'
                        elif l_num_w < r_num_w:
                            w_move = 'left'
                        else:
                            w_move = 'right'
                    else:
                        if l_num_w > r_num_w:
                            w_move = 'left'
                        elif l_num_w < r_num_w:
                            w_move = 'right'
                        else:
                            pass
                elif l_num < r_num:  #R > L
                    if last_l < last_r:
                        w_move = 'right'
                    elif last_l > last_r:
                        if l_num_w > r_num_w:
                            w_move = 'left'
                        elif l_num_w < r_num_w:
                            w_move = 'right'
                        else:
                            w_move = 'left'
                    else:
                        if l_num_w > r_num_w:
                            w_move = 'left'
                        elif l_num_w < r_num_w:
                            w_move = 'right'
                        else:
                            pass
                else:
                    if last_l > last_r:
                        w_move = 'left'
                    elif last_l < last_r:
                        w_move = 'right'
                    else:
                        if l_num_w > r_num_w:
                            w_move = 'right'
                        elif l_num_w < r_num_w:
                            w_move = 'left'
                        else:
                            pass
            elif l_num > 0:
                w_move = 'left'
            elif r_num > 0:
                w_move = 'right'
            else:       #  갭의 경우
                ev3.speaker.beep(1000, 100)
                move(0, -300, 5, stop=False)
                while l_color.rgb()[line_color] > deep_l and r_color.rgb()[line_color] > deep_r and move_measure(0, 55):
                    pass
                left_motor.brake()
                right_motor.brake()
                # 설정거리만큼 후진했는데 검은 선이 없으므로 gap임
                if not move_measure(0, 55):
                    w_move = 'gap'
                elif l_color.rgb()[line_color] <= deep_l and r_color.rgb()[line_color] <= deep_r:    # 라인 이탈한 경우 - 우선순위없는 경우
                    pass
                elif r_color.rgb()[line_color] <= deep_r:    # 라인 이탈한 경우 - 우측
                    w_move = 'right'
                else:                               # 라인 이탈한 경우 - 우선순위없는 경우
                    w_move = 'left'
            print(w_move,  'L: ', l_num, 'R: ', r_num, 'LW: ', l_num_w, 'RW: ', r_num_w)
            print('L', l_list)
            print('R', r_list)
            # wait(1000)
            if w_move == 'left':
                left_motor.run(-500)
                right_motor.run(500)
                while l_color.rgb()[line_color] >= threshold_l:
                    pass
                row_align(20, 50, 10)
            elif w_move == 'right':
                left_motor.run(500)
                right_motor.run(-500)
                while r_color.rgb()[line_color] >= threshold_r:
                    pass
                row_align(20, 50, 10)
            # elif w_move == 'none':
            #     pass
            elif w_move == 'gap':
                move(0, 500, 250, stop=True)     # 250mm만큼 강제전진(gap최대길이:200mm)
                if l_color.rgb()[line_color] < threshold_l or r_color.rgb()[line_color] < threshold_r:    # 라인위에 있음
                    row_align(20, 50, 10)
                else:   # 라인위에 없음
                    move(-50, 500, 1, stop=False)
                    while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(-50, 50):
                        pass
                    left_motor.brake()
                    right_motor.brake()
                    if not move_measure(-50, 50):
                        move(-50, -500, 1, stop=False)
                        while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(-50, 50):
                            pass
                        left_motor.brake()
                        right_motor.brake()
                        move(50, 500, 1, stop=False)
                        while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(50, 50):
                            pass
                        left_motor.brake()
                        right_motor.brake()
                    row_align(20, 50, 10)
            white_detect = 0

        elif aid_kit:
            arm_motor.brake()
            color = r_color

            if aid_kit_detect <= 5:
                aid_kit_detect += 1
                continue
            
            cam_move(1, 1, DISTANCE=150)
            arm_motor.run(1000)
            wait(500)
            move(0, 500, 160)
            arm_motor.brake()
            arm_motor.run_until_stalled(-1000)
            move(0, -500, move_distance((left_motor.angle() + right_motor.angle()) / 2) - 10, stop = False)
            
            while not l_color.rgb()[line_color] < threshold_l or r_color.rgb()[line_color] < threshold_r:
                pass
            left_motor.hold()
            right_motor.hold()
            arm_motor.run(1000)
        if l_line == 14 or r_line ==24:
            aaa=True
            break

            

        elif silver:
            arm_motor.brake()
            beep(3)
            move(0, 500, 150)
            move(0, 500, 40)
            turn = ''

            for i in range(20):
                a, b, c = object_detect(3, DISTANCE=90)
                if a:
                    break

            if not a:
                move(-100, 500, 90)
                wait(100)
                move(0, 500, 25)
                for i in range(20):
                    a, b, c = object_detect(3, DISTANCE=90)
                    if a:
                        break
                if touch.pressed():
                    turn = 'right'

            if not a:
                move(0, -500, 15)
                move(100, 500, 90)
            else:
                move(100, 500, 90)
                move(0, 500, 300)
                move(-100, 500, 90)
                move(-100, 500, 1, stop=False)
                while True:
                    object_bool, x, y = object_detect(3, DISTANCE=90)
                    if object_bool:
                        break

            print(turn)

            if not turn == 'right' and not a :
                move(100, 500, 90)
                wait(100)
                move(0, 500, 25)
                for i in range(20):
                    a, b, c = object_detect(3, DISTANCE=90)
                    if a:
                        break
                if touch.pressed():
                    turn = 'left'
                if not a:
                    move(0, -500, 15)
                    move(-100, 500, 90)
                else:
                    move(-100, 500, 90)
                    move(0, 500, 300)
                    move(100, 500, 90)
                    move(100, 500, 1, stop=False)
                    while True:
                        object_bool, x, y = object_detect(3, DISTANCE=90)
                        if object_bool:
                            break

            if not a:
                loop_int = 0
                if turn == '':
                    loop_int = 0
                elif turn == 'right':
                    loop_int = 90
                else:
                    loop_int = -90
                print(turn)
                print(loop_int)
                if loop_int == 0:
                    move(-100, 500, 90)

                    for i in range(36):
                        move(100, 5000, 5)
                        a, b, c = object_detect(3, DISTANCE = 90)
                        if a:
                            break

                elif loop_int == 90:
                    for i in range(90):
                        move(100, 500, 1)
                        a, b, c = object_detect(3, DISTANCE = 90)
                        if a:
                            break
            left_motor.hold()
            right_motor.hold()

            cam_move(3, 3, touch_mode=True)

            move(0, -500, 75)
            move(100, 500, 180)
            wait(500)
            left_motor.brake()
            right_motor.brake()
            move(0, -200, 50,stop=False)
            wait(3000)
            left_motor.brake()
            right_motor.brake()
            arm_motor.run_until_stalled(500)
            trash_motor.run_until_stalled(200)
            trash_motor.run_until_stalled(-200)


            move(0, 500, 50)
            move(-100, 500, 90)

            move(0, 500, 10, stop = False)
            while not touch.pressed():
                pass
            left_motor.brake()
            right_motor.brake()

            move(0, -500, 90)
            move(50, 500, 43)
            black_check = False
            while True:

                move(0, 500, 1, stop=False)
                while True:
                    if touch.pressed():
                        move(0, -500, 25)
                        move(100, 500, 90)
                        move(0, -500, 1, stop=False)
                        while True:
                            if not move_measure(0, 300):
                                break
                            if l_color.rgb()[line_color] <= threshold_l and r_color.rgb()[line_color] <= threshold_l:
                                distance = move_distance((left_motor.angle() + right_motor.angle()) / 2)
                                move(0, 500, distance)
                                move(100, 500, 180)
                                move(0, 500, distance)
                                black_check = True
                        if not black_check:
                            break
                    if l_color.rgb()[line_color] <= threshold_l and r_color.rgb()[line_color] <= threshold_l or black_check:
                        black_check = False
                        aa = True
                        move(0, 500, 50)
                        move(-50, 500, 1, stop=False)
                        while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(-50, 50):
                            pass
                        left_motor.brake()
                        right_motor.brake()
                        if not move_measure(-50, 50):
                            move(-50, -500, 1, stop=False)
                            while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(-50, 50):
                                pass
                            left_motor.brake()
                            right_motor.brake()
                            move(50, 500, 1, stop=False)
                            while l_color.rgb()[line_color] > threshold_l and r_color.rgb()[line_color] > threshold_l and move_measure(50, 50):
                                pass
                            left_motor.brake()
                            right_motor.brake()
                        row_align(20, 50, 10)
                        break
                if aa:
                    l_list=[]
                    r_list=[]
                    aa=False
                    break
            aa = False
            arm_motor.run_until_stalled(-100)
        elif green:
            print('debug:green:L', l_list)
            print('debug:green:R', r_list)
            if green_detect <= 5:
                green_detect += 1
                continue
            green_detect = 0
            if l_line == 13 and r_line == 23:
                print("move180")
                move(0, 50, 25)
                move(-100, 500, 160, stop=False)
                lb_threshold += 5
                rb_threshold += 5
                while not l_color.rgb()[line_color] < threshold_l or r_color.rgb()[line_color] < threshold_r:
                    pass
                lb_threshold -= 5
                rb_threshold -= 5
                left_motor.hold()
                right_motor.hold()
                continue
            # 그린이 왼쪽이면 True, 오른쪽이면 False
            blacknum = l_list.count(11) if l_line == 13 else r_list.count(21)
            lr = -100 if l_line == 13 else 100
            print(lr)
            print(blacknum)
            move(0, 500, 50, stop=False)
            if blacknum > 4:
                move(0, 500, 50)
                continue
            move(lr, 500, 50, stop=False)
            while not l_color.rgb()[line_color] < threshold_l or r_color.rgb()[line_color] < threshold_r:
                pass
            left_motor.brake()
            right_motor.brake()
            l_list = []
            r_list = []
        # elif touch_:
        #     move(0, -500, 30, stop = True)
        #     move(100, 500, 20, stop = True)
        #     move(0, 500, 30, stop = True)
        #     left_motor.run(100); right_motor.run(700)
        #     while r_color.rgb()[line_color] > threshold_r:
        #         pass
        #     left_motor.hold()
        #     right_motor.hold()
        #     #/
