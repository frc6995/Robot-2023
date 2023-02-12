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

from hid_gamepad import Gamepad

gp = Gamepad(usb_hid.devices)

Col_0 = 0
Col_1 = 1
Col_2 = 2
Row_0 = 18
Row_1 = 19
Row_2 = 20
Row_3 = 21
Row_4 = 22
# Create some buttons. The physical buttons are connected
# to ground on one side and these and these pins on the other.
button_pins = (board.D2, board.D3, board.D4, board.D5)
column_pins = (Col_0, Col_1, Col_2)
row_pins = (Row_0, Row_1, Row_2, Row_3, Row_4)
#   // Set the column pins as inputs and pull them high
#   for ( int j = 0; j < 18; ++j ) {
#     pinMode(Col_0 + j, INPUT_PULLUP);
#   }
#   // Set the row pins as inputs and set them high
#   pinMode (Row_0, OUTPUT);
#   for ( int i = 0; i < 5; ++i ) {
#     pinMode (rowIndices[ i ], OUTPUT);
#     digitalWrite( rowIndices[ i ], HIGH);
#   }

# Map the buttons to button numbers on the Gamepad.
# gamepad_buttons[i] will send that button number when buttons[i]
# is pushed.
gamepad_buttons = (1, 2, 8, 15)

columns = [digitalio.DigitalInOut(pin) for pin in column_pins]
for column in columns:
    column.direction = digitalio.Direction.INPUT
    column.pull = digitalio.Pull.UP

rows = [digitalio.DigitalInOut(pin) for pin in row_pins]
for row in rows:
    row.direction = digitalio.Direction.OUTPU

while True:

    # Buttons are grounded when pressed (.value = False).
    for i, button in range(14):
        gamepad_button_num = gamepad_buttons[i]
        if button.value:
            gp.release_buttons(gamepad_button_num)
            print(" release", gamepad_button_num, end="")
        else:
            gp.press_buttons(gamepad_button_num)
            print(" press", gamepad_button_num, end="")

