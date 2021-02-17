#!/usr/bin/env python

# _7_segment.py
# 2016-12-12
# Public Domain

import time
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html

# For testing purposes each LED is refreshed (REFRESH) for
# 100000 microseconds.  In a practical application you probably
# want to use a figure in the region of 1000 microseconds.

REFRESH=1000

CHARSET={
' ': 0b00000000,
'-': 0b00000010,
'0': 0b11111100,
'1': 0b01100000,
'2': 0b11011010,
'3': 0b11110010,
'4': 0b01100110,
'5': 0b10110110,
'6': 0b00111110,
'7': 0b11100000,
'8': 0b11111110,
'9': 0b11100110,
'A': 0b11101110,
'b': 0b00111110,
'C': 0b10011100,
'c': 0b00011010,
'd': 0b01111010,
'E': 0b10011110,
'F': 0b10001110,
'H': 0b01101110,
'h': 0b00101110,
'L': 0b00011100,
'l': 0b01100000,
'O': 0b11111100,
'o': 0b00111010,
'P': 0b11001110,
'S': 0b10110110,
}

# This defines which gpios are connected to which segments
#          a   b   c   d   e   f   g  dp
SEG2GPIO=[ 6,  5, 21, 19, 13, 26, 12, 16]

# This defines the gpio used to switch on a LED
LED2GPIO=[10,25,24,22,8,7,9,11]

wid = None

showing = [0]*len(LED2GPIO)

CHARS=len(CHARSET)

def display(LED, char):
   if char in CHARSET:
      showing[LED] = CHARSET[char]
   else:
      showing[LED] = 0

def update_display():
   global wid
   wf = []
   for LED in range(len(LED2GPIO)):

      segments = showing[LED] # segments on for current LED

      on = 0 # gpios to switch on
      off = 0 # gpios to switch off

      # set this LED on, others off
      for L in range(len(LED2GPIO)):
         if L == LED:
            on |= 1<<LED2GPIO[L] # switch LED on
         else:
            off |= 1<<LED2GPIO[L] # switch LED off

      # set used segments on, unused segments off
      for b in range(8):
         if segments & 1<<(7-b):
            on |= 1<<SEG2GPIO[b] # switch segment on
         else:
            off |= 1<<SEG2GPIO[b] # switch segment off

      wf.append(pigpio.pulse(on, off, REFRESH))

      print(on, off, REFRESH) # debugging only

   pi.wave_add_generic(wf) # add pulses to waveform
   new_wid = pi.wave_create() # commit waveform
   pi.wave_send_repeat(new_wid) # transmit waveform repeatedly

   if wid is not None:
      pi.wave_delete(wid) # delete no longer used waveform

   print("wid", wid, "new_wid", new_wid)

   wid = new_wid

pi = pigpio.pi()

# Set all used gpios as outputs.

for segment in SEG2GPIO:
   pi.set_mode(segment, pigpio.OUTPUT)

for LED in LED2GPIO:
   pi.set_mode(LED, pigpio.OUTPUT)

char=0

ck = CHARSET.keys()

#while True:
if True:

   # To test loop over character set.

   display(0,'1')
   display(1,'2')
   display(2,'3')
   display(3,'4')
   display(4,'5')
   display(5,'6')
   display(6,'7')
   display(7,'8')
   update_display()

   time.sleep(5)

   display(0,'8')
   display(1,'7')
   display(2,'6')
   display(3,'5')
   display(4,'4')
   display(5,'3')
   display(6,'2')
   display(7,'1')
   update_display()

   time.sleep(5)

   display(0,'-')
   display(1,'-')
   display(2,'-')
   display(3,'-')
   display(4,'-')
   display(5,'-')
   display(6,'-')
   display(7,'-')
   update_display()

pi.wave_delete(wid)

pi.stop()
