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

# gpio pin configuration for rpints
power_relay = 29 # GPIO 5
dummy3_power = 31 # GPIO 6
tacho_power = 12 # GPIO 18
speedo_power = 11 # GPIO 17
auto_mode_out = 36 # GPIO 16

# gpio pin configurations for brewpi
msgctr_power = 36 # GPIO 16
auto_mode_in = 31 # GPIO 6

class PANPState(Enum):
    AUTO = 13 # GPIO 27
    NORM = 40 # GPIO 21
    PURSUIT = 37 # GPIO 26


class PANPButton(Enum):
    AUTO = 19 # GPIO 10
    NORM = 21 # GPIO 9
    PURSUIT = 26 # GPIO 7


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
            GPIO.output(dummy3_power, 1)
            GPIO.output(tacho_power, 1)
            GPIO.output(speedo_power, 1)
            GPIO.output(auto_mode_out, 1)
            self.state = PANPState.AUTO
        elif channel is PANPButton.NORM.value:
            GPIO.output(dummy3_power, 0)
            GPIO.output(tacho_power, 0)
            GPIO.output(speedo_power, 0)
            GPIO.output(auto_mode_out, 0)
            self.state = PANPState.NORM
        elif channel is PANPButton.PURSUIT.value:
            GPIO.output(dummy3_power, 0)
            GPIO.output(tacho_power, 0)
            GPIO.output(speedo_power, 0)
            GPIO.output(auto_mode_out, 0)
            self.state = PANPState.PURSUIT
        GPIO.output(old_state.value,1)
        GPIO.output(self.state.value,0)

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
    for p in PANPState:
        GPIO.output(p.value, 1)
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

# set up the gpio system
GPIO.setmode(GPIO.BOARD)

# disable the power button by opening the relay
GPIO.setup(power_relay, GPIO.OUT, initial=1)

# allow boot to continue
n = sdnotify.SystemdNotifier()
n.notify("READY=1")

my_hostname=socket.gethostname()
n.notify("STATUS=PANP service running on {0}".format(my_hostname))

if my_hostname == 'rpints':
    # initailize the dashboard power relays
    GPIO.setup(dummy3_power, GPIO.OUT, initial=1)
    GPIO.setup(tacho_power, GPIO.OUT, initial=1)
    GPIO.setup(speedo_power, GPIO.OUT, initial=1)

    # set up output pin for brewpi
    GPIO.setup(auto_mode_out, GPIO.OUT, initial=0)

    # set up the panp button handler
    h = PANPHandler()

elif my_hostname == 'brewpi':
    # initailize the message center power relay
    GPIO.setup(msgctr_power, GPIO.OUT, initial=0)

    # set up input pin for auto mode from rpints
    GPIO.setup(auto_mode_in, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

else:
    # fail with an error
    n.notify("Unknown hostname, exiting")
    n.notify("ERRNO=1")
    sys.exit(1)

# sleep forever waiting for events
while True:
    time.sleep(1)
