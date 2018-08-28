#!/usr/bin/env python3

from utils import gps2utc
from pymavlink import mavutil
import json
import os

def convert_dataflash_to_json(dataflash_file, outfile, sectionfile):
  print("Openning log file %s" % dataflash_file)
  mavmaster = mavutil.mavlink_connection(dataflash_file)

  print("Parsing messages")
  mavmsg = mavmaster.recv_msg()

  # drop all data prior to first GPS message to sync boot time
  boot_time = -1
  data = {}
  sections = []
  section_start = 0
  inair = False
  last_status = 0

  while mavmsg is not None:
    msg_type = mavmsg.get_type()
    msg = mavmsg.to_dict()
        
    if msg_type == 'GPS' and msg['Status'] > 3 and boot_time == -1:
      # GPS Status can be:
      # 0 - no gps
      # 1 - gps detected, but no lock
      # 2 - 2D lock
      # 3 - 3D lock
      # 4 - 3D 'DGPS' (differential improvements?)
      # 5 - 3D RTK Float
      # 6 - 3D RTK Fixed
      # Since we only need time, status can be anything above 1
      print("Found first good (status > 3) GPS message")
      print("GPS time is: %f" % gps2utc(msg['GMS']/1000., msg['GWk']))
      print("Uptime(uS) is: %d" % msg['TimeUS'])
      boot_time = gps2utc(msg['GMS']/1000., msg['GWk']) - msg['TimeUS']/1000000.
      section_start = boot_time
      print("Setting boot time to %f" % boot_time)
      print("Setting home position")
      data['HOME'] = {'stamp':{'Lat':msg['Lat'], 'Lng':msg['Lng'], 'Alt':msg['Alt']}}

    if boot_time != -1:
      if msg_type == 'CURR':
        if msg['Curr'] > 5 and not inair:
          stamp = boot_time + msg['TimeUS'] / 1000000.
          sections.append({'start':section_start, 'end':stamp, 'inair':False})
          section_start = stamp
          inair = True
        elif msg['Curr'] < 5 and inair:
          stamp = boot_time + msg['TimeUS'] / 1000000.
          sections.append({'start':section_start, 'end':stamp, 'inair':True})
          inair = False
      elif msg_type == 'GPS':
        boot_time = gps2utc(msg['GMS']/1000., msg['GWk']) - msg['TimeUS']/1000000.
        last_status = int(msg['Status'])
      stamp = boot_time + msg['TimeUS'] / 1000000.
      if msg_type not in data:
        data[msg_type] = {}
      if last_status > 5:
        data[msg_type][stamp] = msg
    mavmsg = mavmaster.recv_msg()
  with open(outfile, 'w') as f:
    json.dump(data,f, indent=2, sort_keys=True)
  with open(sectionfile, 'w') as f:
    json.dump(sections,f,indent=2,sort_keys=True)

if __name__ == '__main__':
  import sys
  if len(sys.argv) < 4:
    print("Too few arguments. Usage: %s dataflash_log output_json selection_json" % sys.argv[0], file=sys.stderr)
    sys.exit(1)
  convert_dataflash_to_json(sys.argv[1], sys.argv[2], sys.argv[3])
