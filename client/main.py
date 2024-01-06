import struct
import serial
import time
import sys
# from evdev import InputDevice, list_devices, ecodes
import pygame
from pygame.locals import *
import os

CMD_MOVE_TO = 0
CMD_SET_SPEED = 1
CMD_SET_ACC = 2
CMD_RESET_POS = 3

float_cmds = {CMD_SET_SPEED, CMD_SET_ACC}

portFile = '/dev/' + next(filter(lambda a: a.startswith('ttyUSB'), os.listdir('/dev')))
print(portFile)
port = serial.Serial(portFile, 115200)


def send_command(cmd, motor, arg: int | float):
    if cmd in float_cmds:
        data = struct.pack('<BBf', cmd, motor, float(arg))
    else:
        data = struct.pack('<BBi', cmd, motor, int(arg))

    print(f'sending {data}')
    port.write(data)
    # while(port.readall())


accs = [50, 100, 150, 200]


# found_controller = False
# devices = [InputDevice(fn) for fn in list_devices()]
# for dev in devices:
#   if dev.name == 'Wireless Controller':
#     found_controller = True
#     break
# if not found_controller:
#   print('Connect controller you moron')
#   exit()
#
# for e in dev.read_loop():
#     if e.type == ecodes.EV_ABS and e.code == 1:
#         print(e)

pygame.init()
print(pygame.joystick.get_count())
joy = pygame.joystick.Joystick(0)

joy.get_axis(1)

axis_steppers = {
    1: 3,
    3: 1,
}

prev_axis_val = {k: None for k in axis_steppers}

button_steppers = {
    0: 0,
    1: 2
}

# button_pressed_coord = 100
# button_released_coord = 0

# axis_min = 0
# axis_max = 200*5
# axis_config = {
#     3: (0, -1000),
#     1: (0, 1000),
# }

axis_config = {
    3: (0, -280),
    1: (0, 280),
}

button_config = {
    0: (0, 100),
    1: (1, -100),
}






while (cl := port.readline().decode().strip()) != 'READY!':
    print(cl)
print(cl)
print('Got ready, sending shit')

# for i in range(4):
#     send_command(CMD_SET_ACC, i, 400)
#     send_command(CMD_SET_SPEED, i, 3200)
    # send_command(CMD_MOVE_TO, i, 1000000* (-1**(i+1)))

# while(True):
#     try:
#         sys.stdout.write(port.read(1).decode())
#     except UnicodeDecodeError:
#         print('X', end='')


def map_vals(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


while True:
    for event in [pygame.event.wait(), ] + pygame.event.get():
        if event.type in (JOYBUTTONUP, JOYBUTTONDOWN):
            btn = event.button
            if btn in button_steppers:
                dst = button_config[btn][1] if event.type == JOYBUTTONDOWN else button_config[btn][0]
                send_command(CMD_MOVE_TO, button_steppers[btn], dst)
            # else:
            #     print('Entering read loop')
            #     while True:
            #         print(port.readline())
        elif event.type == JOYAXISMOTION and (axis := event.axis) in axis_steppers:
            v = event.value
            if prev_axis_val[axis] is not None and abs(v - prev_axis_val[axis]) < 0.1:
                continue
            prev_axis_val[axis] = v

            dst = map_vals(event.value, -1, 1, axis_config[axis][0], axis_config[axis][1])
            send_command(CMD_MOVE_TO, axis_steppers[axis], dst)
