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

# ==================================== Functions ==================================== #

# 중복 계산을 피하기위해 계산식을 함수로 만듦
def color_line(color, in_value):     # color: 'black', 'green', 'white', 'red', silver'
    if in_value.count(0) > 0:   # 입력된튜플값 중 0이 있으면 1로 바꿈
        in_value = list(in_value)
        for z in range(3):
            if in_value[z] == 0:
                in_value[z] = 1
        in_value = tuple(in_value)
    if color == 'silver':
        value = sum(in_value)
    elif color == 'white':
        value = sum(in_value)+in_value[0]+in_value[1]+(in_value[1]/in_value[2])
    elif color == 'green':
        value = ((in_value[1] / sum(in_value)) * 100) - ((in_value[0] / (in_value[0]+in_value[1])) * 100)
    elif color == 'red':
        value = (in_value[0] / sum(in_value)) * 100
    elif color == 'black':
        value = in_value[0]+in_value[1]+(in_value[1]/in_value[2])
    return value

# R, G, B로 분석할 기준 값 출력 함수
def color_reference(rgb_data):
    color_dict = {}
    # white 합계와 silver 합계의 경계값을 silver 기준, 측정값의 합계가 기준보다 크면 silver
    color_dict['l_silver_line'] = (color_line('silver', rgb_data['l_silver'])+color_line('silver', rgb_data['l_white'])) / 1.8
    color_dict['r_silver_line'] = (color_line('silver', rgb_data['r_silver'])+color_line('silver', rgb_data['r_white'])) / 1.8
    # red의 비중 -10 이상이면 red이다
    color_dict['l_red_line'] = color_line('red', rgb_data['l_red']) - 10
    color_dict['r_red_line'] = color_line('red', rgb_data['r_red']) - 10
    # (green의 비중-(r/(r+g))*100)-5 값이 ??% 이상이면 green이다.
    color_dict['l_green_line'] = color_line('green', rgb_data['l_green']) - 10
    color_dict['r_green_line'] = color_line('green', rgb_data['r_green']) - 10
    # white 기준 -10을 기준, 이것보다 높으면 흰색
    color_dict['l_white_line'] = color_line('white', rgb_data['l_white']) - 5
    color_dict['r_white_line'] = color_line('white', rgb_data['r_white']) - 5
    # black의 green 값 + 5를 기준, 이것보다 낮으면 black
    color_dict['l_black_line'] = color_line('black', rgb_data['l_black']) + 5
    color_dict['r_black_line'] = color_line('black', rgb_data['r_black']) + 5
    # 경계값 구하기
    threshold_l = (rgb_data['l_white'][1] + rgb_data['l_black'][1]) / 2
    threshold_r = (rgb_data['r_white'][1] + rgb_data['r_black'][1]) / 2
    deep_l = rgb_data['l_black'][1] + 3
    deep_r = rgb_data['r_black'][1] + 3
    return color_dict, threshold_l, threshold_r, deep_l, deep_r

def color_detect(color_dict):
    # l_color, r_color 센서 측정값 분석 - R,G,B 측정값이 추가되면 같이 추가해야 함
    l_rgb = l_color.rgb()
    r_rgb = r_color.rgb()
    l_sensor = l_rgb[1]     # RGB값중 green값으로 라인트레이싱
    r_sensor = r_rgb[1]     # RGB값중 green값으로 라인트레이싱
    # 각 색 기준값과 현재 left 센서 측정값 비교 - l_line 결과값
    if color_line('silver', l_rgb) > color_dict['l_silver_line']:
        l_line = 'silver'
    elif color_line('red', l_rgb) > color_dict['l_red_line']:
        l_line = 'red'
    elif color_line('green', l_rgb) > color_dict['l_green_line']:
        l_line = 'green'
    elif color_line('white', l_rgb) > color_dict['l_white_line']:
        l_line = 'white'
    elif color_line('black', l_rgb) < color_dict['l_black_line']:
        l_line = 'black'
    else:
        l_line = 'none'
    # 각 색 기준값과 현재 right 센서 측정값 비교 - r_line 결과값
    if color_line('silver', r_rgb) > color_dict['r_silver_line']:
        r_line = 'silver'
    elif color_line('red', r_rgb) > color_dict['r_red_line']:
        r_line = 'red'
    elif color_line('green', r_rgb) > color_dict['r_green_line']:
        r_line = 'green'
    elif color_line('white', r_rgb) > color_dict['r_white_line']:
        r_line = 'white'
    elif color_line('black', r_rgb) < color_dict['r_black_line']:
        r_line = 'black'
    else:
        r_line = 'none'
    return l_line, r_line, l_sensor, r_sensor

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
            st_B = ((speed*2)/100)*steering+speed
            st_C = speed
            moving = 'left'
        elif 0 < steering <= 100:
            st_B = speed
            st_C = -((speed*2)/100)*steering+speed
            moving = 'right'
        elif 100 < steering:
            st_B = speed
            TURN = -speed
            moving = 'right'
    if moving == 'straight':
        left_motor.run(speed+move_set['weight'])
        right_motor.run(speed-move_set['weight'])
        while (abs(left_motor.angle())+abs(right_motor.angle()))/2 <= move_mm:
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

# ==================================== Preferences ==================================== #

# RGB 측정값 입력
rgb_data = {
    "l_black": (6,3,11),
    "r_black": (5,3,8),
    "l_silver": (52,59,100),
    "r_silver": (39,20,100),
    "l_red": (20,1,4),
    "r_red": (22,2,5),
    "l_green": (3,6,4),
    "r_green": (6,10,5),
    "l_white": (16,12,41),
    "r_white": (16,14,37)
}

# weight: 우측으로 휘면 -값, 좌측으로 휘면 +값, wheel_diameter: 바퀴지름, axle_track: 바퀴간 거리
move_set = {'weight': -2, 'wheel_diameter': 39.75, 'axle_track': 168.7}

# 라인트레이싱 구동함수 교체 경계값설정
speed_threshold = 100

# 라인트레이싱 설정값 정의 dc: 0~100%, run: deg/s
dc_mode = {'speed':70, 'kp':21, 'ki':1.123456789, 'kd':3.456789, 'min_speed':50, 'st_ratio':8, 'mode':'dc'}
run_mode = {'speed':500, 'kp':15, 'ki':0.002, 'kd':30, 'min_speed': 135, 'st_ratio':8,'mode':'run'}
# ==================================== Main Loop ==================================== #

# rgb_data로 색상 측정의 기준값 만듦
color_dict, threshold_l, threshold_r, deep_l, deep_r = color_reference(rgb_data)
# pid_control() 함수에 사용할 변수
integral = 0
last_error = 0
max_error = ((rgb_data['l_white'][1] - threshold_l) + (rgb_data['r_white'][1] - threshold_r)) / 2

start = 0
while True:
    # EV3 center 버튼을 누를 때까지 대기 - 시작
    message("Press CENTER button", "to START")
    while not Button.CENTER in ev3.buttons.pressed():
        pass
    # 출발 위치가 T자일 경우를 대비해서 최초 1번 0.2초 강제전진
    if start == 0:
        start = 1
        move(0, 500, 24, 0, stop=False)     # 약 24mm전진
    # EV3 down 버튼을 누를 때까지 대기 - 정지
    message("Press DOWN button", "to STOP")
    # main 프로그램 시작 EV3 다운버튼이 눌리면 종료
    # 라인따라가는 정보저장 초기화
    l_list = []
    r_list = []
    while True:
        # 라인트레이싱이 종료되면 다양한 처리를 하고 다시 라인트레이싱으로 복귀
        while True:
            # 컬러센서의 색상 측정 값 및 rgb()함수의 초록값 
            l_line, r_line, l_sensor, r_sensor = color_detect(color_dict)
            # 현재 두개의 컬러센서가 측정하고 있는 색상 값 저장
            l_list.append(l_line)
            r_list.append(r_line)
            if len(l_list) > 10:
                del l_list[0]
                del r_list[0]
            # 다운 버튼을 눌르면 True, down 버튼 클릭하면 루프 종료
            pressed = Button.DOWN in ev3.buttons.pressed()
            # 좌측, 우측 중 하나 또는 두개가 white, green, silver, red이면 True
            white = l_line == 'white' and r_line == 'white'
            green = l_line == 'green' or r_line == 'green'
            silver = l_line == 'silver' and r_line == 'silver'
            red = l_line == 'red' or r_line == 'red'
            # 선따라가기 루프 종료 - down 버튼 클릭, green, red, silver
            if pressed or white or green or red or silver:
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
        print(l_list)
        print(r_list)
        # 사용자 중지로 처음으로 돌아감
        if pressed:
            break

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