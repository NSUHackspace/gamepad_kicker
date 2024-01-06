import struct
from typing import Any

import serial
import time
import sys
# from evdev import InputDevice, list_devices, ecodes
import pygame
from pygame.locals import *
import json
import os

CMD_MOVE_TO = 0
CMD_SET_SPEED = 1
CMD_SET_ACC = 2
CMD_RESET_POS = 3

float_cmds = {CMD_SET_SPEED, CMD_SET_ACC}

# portFile = '/dev/' + next(filter(lambda a: a.startswith('ttyUSB'), os.listdir('/dev')))
portFile = sys.argv[1]
print(portFile)
port = serial.Serial(portFile, 115200)
config = json.load(open(sys.argv[2]))


def send_command(cmd, motor, arg: int | float):
    if cmd in float_cmds:
        data = struct.pack('<BBf', cmd, motor, float(arg))
    else:
        data = struct.pack('<BBi', cmd, motor, int(arg))

    print(f'sending {data}')
    port.write(data)
    # while(port.readall())

pygame.init()
print(pygame.joystick.get_count())

for i in range(pygame.joystick.get_count()):
    joy = pygame.joystick.Joystick(i)
    print(joy.get_name())
    if joy.get_name() == config['controller']:
        break
    joy.quit()


def keys_to_int(d: dict[str, Any]) -> dict[int, Any]:
    return {int(k): v for k, v in d.items()}


axis_steppers = keys_to_int(config['axis_steppers'])
axis_config = keys_to_int(config['axis_config'])
button_steppers = keys_to_int(config['button_steppers'])
button_config = keys_to_int(config['button_config'])


prev_axis_val = {k: None for k in axis_steppers}



while (cl := port.readline().decode().strip()) != 'READY!':
    print(cl)
print(cl)
print('Got ready, sending shit')


def map_vals(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


while True:
    for event in [pygame.event.wait(), ] + pygame.event.get():
        if event.type == pygame.QUIT:
            exit(0)
        if event.type in (JOYBUTTONUP, JOYBUTTONDOWN):
            btn = event.button
            if btn in button_steppers:
                dst = button_config[btn][1] if event.type == JOYBUTTONDOWN else button_config[btn][0]
                send_command(CMD_MOVE_TO, button_steppers[btn], dst)
            elif btn == config['quit_button']:
                pygame.quit()
                exit(0)
            # else:
            #     print('Entering read loop')
            #     while True:
            #         print(port.readline())
        elif event.type == JOYAXISMOTION and (axis := event.axis) in axis_steppers:
            v = event.value
            # if axis == 1:
                # print(v)
            if prev_axis_val[axis] is not None and abs(v - prev_axis_val[axis]) < 0.1:
                continue
            prev_axis_val[axis] = v


            dst = map_vals(event.value, -1, 1, axis_config[axis][0], axis_config[axis][1])
            send_command(CMD_MOVE_TO, axis_steppers[axis], dst)
