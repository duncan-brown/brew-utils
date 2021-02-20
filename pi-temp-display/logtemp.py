#!/usr/bin/env python

import os
import time

MASH_PROBE = '/sys/bus/w1/devices/28-012052b65be5/w1_slave'

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

start_time = time.time()

while True:
   tempvalue_f = get_probe_temp(MASH_PROBE)
   tempvalue = round(tempvalue_f,1)
   minutes = round((time.time() - start_time)/60.0,2)
   print minutes, tempvalue
   time.sleep(30)
