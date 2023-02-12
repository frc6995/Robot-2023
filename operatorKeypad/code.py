# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# You must add a gamepad HID device inside your boot.py file
# in order to use this example.
# See this Learn Guide for details:
# https://learn.adafruit.com/customizing-usb-devices-in-circuitpython/hid-devices#custom-hid-devices-3096614-9

import board
import digitalio
import analogio
import usb_hid

from adafruit_hid.hid_gamepad import Gamepad

gp = Gamepad(usb_hid.devices)

# microcontroller GPIO pins
Col_0 = board.GP0
Col_1 = board.GP1
Col_2 = board.GP2
Row_0 = board.GP18
Row_1 = board.GP19
Row_2 = board.GP20
Row_3 = board.GP21
Row_4 = board.GP22
# Create some buttons. The physical buttons are connected
# to ground on one side and these pins on the other.
column_pins = (Col_0, Col_1, Col_2)
row_pins = (Row_0, Row_1, Row_2, Row_3, Row_4)

# Map the buttons to button numbers on the Gamepad.
# We use 15 for the missing middle button to maintain continuity on the ones we do use
gamepad_buttons = (
    ( 1,  2,  3),
    ( 4,  5,  6),
    ( 7,  8,  9),
    (10, 11, 12),
    (13, 15, 14))

# configure the columns as input pins and rows as outputs
columns = [digitalio.DigitalInOut(pin) for pin in column_pins]
for column in columns:
    column.direction = digitalio.Direction.INPUT
    column.pull = digitalio.Pull.UP

rows = [digitalio.DigitalInOut(pin) for pin in row_pins]
for row in rows:
    row.direction = digitalio.Direction.OUTPUT
# fill 16-value array with Trues
button_state = [True for i in range(16)]
print('start')
while True:
    
    for i, row in enumerate(rows):
        # send power to this row (idk why that's False)
        row.value = False
        # check each column and set the button state for those receiving power
        # column.value is false if button pressed
        for j, column in enumerate(columns):
            if column.value:
                button_state[gamepad_buttons[i][j]] = False
            else:
                button_state[gamepad_buttons[i][j]] = True
        # cut power to this row
        row.value = True
    # send the current state
    for i, state in enumerate(button_state):
        # 0 is invalid button id
        if i == 0:
            continue
        if state:
            gp.press_buttons(i)
        else:
            gp.release_buttons(i)

