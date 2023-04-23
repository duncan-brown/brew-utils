#!/usr/bin/python

import os
import sys
import time
import socket
import signal
import sdnotify
import RPi.GPIO as GPIO
from enum import Enum

GPIO.cleanup()

# common gpio pins
serial_enable = 22 # GPIO 25

# gpio pin configuration for rpints
power_relay = 16      # GPIO 23
lower_dash_power = 19 # GPIO 10 (MOSI)
upper_dash_power = 12 # GPIO 18
sp_power = 11         # GPIO 17
pursuit_mode_out = 15 # GPIO 22

# gpio pin configurations for brewpi
msgctr_power = 36    # GPIO 16
pursuit_mode_in = 15 # GPIO 22

class PANPState(Enum):
    AUTO = 35    # GPIO 19
    NORM = 38    # GPIO 20
    PURSUIT = 32 # GPIO 12


class PANPButton(Enum):
    AUTO = 29    # GPIO 5
    NORM = 31    # GPIO 6
    PURSUIT = 36 # GPIO 16


class PANPHandler:
    # current state
    state = None
    last_push = None

    # initalize panp lamps on boot
    def __init__(self):
        for p in PANPState:
            GPIO.setup(p.value, GPIO.OUT, initial=1)
        self.state = PANPState.AUTO
        GPIO.output(self.state.value,0)
        for p in PANPButton:
            GPIO.setup(p.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(p.value, GPIO.FALLING, callback=self.button, bouncetime=50)

    def change_state(self, channel):
        old_state = self.state
        if channel is PANPButton.AUTO.value:
            GPIO.output(lower_dash_power, 1)
            GPIO.output(upper_dash_power, 1)
            GPIO.output(sp_power, 1)
            GPIO.output(pursuit_mode_out, 0)
            self.state = PANPState.AUTO
        elif channel is PANPButton.NORM.value:
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 1)
            GPIO.output(pursuit_mode_out, 0)
            self.state = PANPState.NORM
        elif channel is PANPButton.PURSUIT.value:
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 0)
            GPIO.output(pursuit_mode_out, 1)
            self.state = PANPState.PURSUIT
        GPIO.output(old_state.value,0)
        GPIO.output(self.state.value,1)

    def button(self, channel):
        if channel is not self.last_push:
            time.sleep(0.05)
            if GPIO.input(channel) == GPIO.LOW:
                self.change_state(channel)
                self.last_push = channel


# set up exit signal handler for systemd
def sigterm_handler(_signo, _stack_frame):
    msg="PANP service on {0} exiting on SIGTERM".format(my_hostname)
    print(msg)
    n.notify("STATUS={0}".format(msg))
    n.notify("STOPPING=1")
    if my_hostname == 'rpints':
        for p in PANPState:
            GPIO.output(p.value, 0)
        GPIO.output(lower_dash_power, 1)
        GPIO.output(upper_dash_power, 1)
        GPIO.output(sp_power, 1)
        GPIO.output(pursuit_mode_out, 0)
    else:
        GPIO.output(msgctr_power,0)
    GPIO.output(serial_enable,0)
    sys.exit(0)

# install the signal handler
signal.signal(signal.SIGTERM, sigterm_handler)

# set up the gpio system
GPIO.setmode(GPIO.BOARD)

# disable the power button by opening the relay
GPIO.setup(power_relay, GPIO.OUT, initial=1)

# enable the 3.3V to 5V serial converter
GPIO.setup(serial_enable, GPIO.OUT, initial=1)

# allow boot to continue
n = sdnotify.SystemdNotifier()
n.notify("READY=1")

my_hostname=socket.gethostname()
n.notify("STATUS=PANP service running on {0}".format(my_hostname))

if my_hostname == 'rpints':
    # initailize the dashboard power relays
    GPIO.setup(lower_dash_power, GPIO.OUT, initial=0)
    GPIO.setup(upper_dash_power, GPIO.OUT, initial=0)
    GPIO.setup(sp_power, GPIO.OUT, initial=0)

    # set up output pin for brewpi
    GPIO.setup(pursuit_mode_out, GPIO.OUT, initial=0)

    # set up the panp button handler
    h = PANPHandler()

elif my_hostname == 'brewpi':
    # initailize the message center power relay
    GPIO.setup(msgctr_power, GPIO.OUT, initial=0)

    # set up input pin for auto mode from rpints
    GPIO.setup(pursuit_mode_in, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

else:
    # fail with an error
    n.notify("Unknown hostname, exiting")
    n.notify("ERRNO=1")
    sys.exit(1)

# sleep forever waiting for events
while True:
    time.sleep(1)
