#!/usr/bin/env python3

from utils import gps2utc
import sys
import quaternion
from math import radians
import utm

def convert_dataflash_to_csv(dataflash_file, outfile, pos='NKF'):
  from pymavlink import mavutil
  print("Current implementation only reads position and attitude from EKF")
  print("Openning log file %s" % dataflash_file)
  mavmaster = mavutil.mavlink_connection(dataflash_file)

  print("Opening output file %s" % outfile)
  f = open(outfile, 'w')
  
  print("Parsing messages")
  mavmsg = mavmaster.recv_msg()

  # drop all data prior to first GPS message to sync boot time
  boot_time = -1

  latest_q = None

  while mavmsg is not None:
    msg_type = mavmsg.get_type()
    msg = mavmsg.to_dict()
    print(msg_type)
    if boot_time != -1:
      if msg_type == 'NKF1':
        stamp = boot_time + msg['TimeUS'] / 1000000.
        x = msg['PN']
        y = msg['PE']
        z = msg['PD']
        # In a NED down frame:
        # x = roll
        # y = pitch
        # z = yaw
        yaw_rad = radians(msg['Yaw'])
        pitch_rad = radians(msg['Pitch'])
        roll_rad = radians(msg['Roll'])
        latest_q = quaternion.from_euler_angles(roll_rad, pitch_rad, yaw_rad)
        if pos == 'nkf':
          line = ('%f,%f,%f,%f,%f,%f,%f,%f\n') % (stamp, x, y, z, latest_q.x, latest_q.y, latest_q.z, latest_q.w)
          f.write(line)
      elif pos=='gps' and msg_type == 'GPS':
        stamp = gps2utc(msg['GMS']/1000., msg['GWk'])
        utm_pos = utm.from_latlon(msg['Lat'], msg['Lng'])
        x = utm_pos[1]
        y = utm_pos[0]
        z = -msg['Alt']
        if latest_q is not None:
          line = ('%f,%f,%f,%f,%f,%f,%f,%f\n') % (stamp, x, y, z, latest_q.x, latest_q.y, latest_q.z, latest_q.w)
          latest_q = None
          f.write(line)
        
    elif msg_type == 'GPS' and msg['Status'] > 1:
      # GPS Status can be:
      # 0 - no gps
      # 1 - gps detected, but no lock
      # 2 - 2D lock
      # 3 - 3D lock
      # 4 - 3D 'DGPS' (differential improvements?)
      # 5 - 3D RTK Float
      # 6 - 3D RTK Fixed
      # Since we only need time, status can be anything above 1
      print("Found first good (status > 1) GPS message")
      print("GPS time is: %f" % gps2utc(msg['GMS']/1000., msg['GWk']))
      print("Uptime(uS) is: %d" % msg['TimeUS'])
      boot_time = gps2utc(msg['GMS']/1000., msg['GWk']) - msg['TimeUS']/1000000.
      print("Setting boot time to %f" % boot_time)
    mavmsg = mavmaster.recv_msg()
  f.close()

if __name__ == '__main__':
  if len(sys.argv) < 3:
    print("Too few arguments. Usage: %s [dataflash log] [output csv]" % sys.argv[0], file=sys.stderr)
    sys.exit(1)
  if len(sys.argv) > 3:
    pos_type = sys.argv[3]
  else:
    pos_type = 'gps'
  convert_dataflash_to_csv(sys.argv[1], sys.argv[2], pos=pos_type)
