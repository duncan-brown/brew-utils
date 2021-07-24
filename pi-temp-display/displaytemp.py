#!/usr/bin/env python

# _7_segment.py
# 2016-12-12
# Public Domain

import pigpio # http://abyz.co.uk/rpi/pigpio/python.html
import os
import time
from threading import Thread
from multiprocessing import Queue

KEEZER_PROBES = ['/sys/bus/w1/devices/28-012058fc2851/w1_slave',
                 '/sys/bus/w1/devices/28-012058fbceb5/w1_slave',
                 '/sys/bus/w1/devices/28-012058f936f3/w1_slave',
                 '/sys/bus/w1/devices/28-012052ba8dab/w1_slave',
                 '/sys/bus/w1/devices/28-012052b92541/w1_slave',
                 '/sys/bus/w1/devices/28-012052b426ca/w1_slave']

MASH_PROBE = '/sys/bus/w1/devices/28-012052b65be5/w1_slave'
HLT_PROVE = '/sys/bus/w1/devices/28-0120529d8f20/w1_slave'

# For testing purposes each LED is refreshed (REFRESH) for
# 100000 microseconds.  In a practical application you probably
# want to use a figure in the region of 1000 microseconds.
REFRESH=500

CHARSET={
' ': 0b00000000,
'-': 0b00000010,
'0': 0b11111101,
'1': 0b01100001,
'2': 0b11011011,
'3': 0b11110011,
'4': 0b01100111,
'5': 0b10110111,
'6': 0b10111111,
'7': 0b11100001,
'8': 0b11111111,
'9': 0b11100111,
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
LED2GPIO=[23,22,18,27,24,25,8,7]

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

   pi.wave_add_generic(wf) # add pulses to waveform
   new_wid = pi.wave_create() # commit waveform
   if wid is not None:
      pi.wave_delete(wid) # delete no longer used waveform
      pi.wave_tx_stop() # stop transmission
   pi.wave_send_repeat(new_wid) # transmit waveform repeatedly
   wid = new_wid
  
def get_probe_temp(probe):
   fileobj = open(probe,'r')
   lines = fileobj.readlines()
   fileobj.close()
   status = lines[0][-4:-1]
   equals_pos = lines[1].find('t=')
   tempstr = lines[1][equals_pos+2:]
   tempvalue_c = float(tempstr)/1000.0
   tempvalue_f = tempvalue_c * 9.0 / 5.0 + 32.0
   return tempvalue_f

def get_keezer_temps(out_q):
   while True:
      try:
         keezer_temps = []
         for k in KEEZER_PROBES:
            tempvalue_f = get_probe_temp(k)
            tempvalue = int(round(tempvalue_f,0))
            keezer_temps.append(tempvalue)
            
         k_min = str(min(keezer_temps)).rjust(2)
         k_max = str(max(keezer_temps)).rjust(2)
         display_str=k_max+k_min
      except:
         display_str = '----'

      out_q.put(display_str)
      time.sleep(1)

def get_mash_temp(out_q):
   while True:
      try:
         tempvalue_f = get_probe_temp(MASH_PROBE)
         tempvalue = round(tempvalue_f,1)
         display_str = ''.join(str(tempvalue).split('.')).rjust(4)
      except:
         display_str = '----'

      out_q.put(display_str)
      time.sleep(1)

def display_temps(keezer_q, mash_q):
   while True:
      try:
         keezer_str = keezer_q.get(False)
         for d in range(len(keezer_str)):
             display(d + 4, keezer_str[d])
         update_display()
      except:
         pass

      try:
         mash_str = mash_q.get(False)
         for d in range(len(mash_str)):
             display(d, mash_str[d])
         update_display()
      except:
         pass

      time.sleep(1)


# initialize the gpio controller
pi = pigpio.pi()

# Set all used gpios as outputs.
for segment in SEG2GPIO:
   pi.set_mode(segment, pigpio.OUTPUT)

for LED in LED2GPIO:
   pi.set_mode(LED, pigpio.OUTPUT)

# initialize the display
for d in range(len(LED2GPIO)):
   display(d,'-')
update_display()

# create ipc queues
keezer_q = Queue()
mash_q = Queue()

t1 = Thread(target=get_keezer_temps, args=(keezer_q,))
t2 = Thread(target=get_mash_temp, args=(mash_q,))
t3 = Thread(target=display_temps, args=(keezer_q,mash_q,))

t1.daemon = True
t2.daemon = True
t3.daemon = True

t1.start()
t2.start()
t3.start()

try:
   while True:
      time.sleep(0.1)
except KeyboardInterrupt:
   for d in range(len(LED2GPIO)):
      display(d,' ')
   update_display()

   # clean up on exit
   pi.wave_delete(wid)
   pi.stop()

   raise KeyboardInterrupt

