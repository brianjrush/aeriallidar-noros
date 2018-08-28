#!/usr/bin/env python3

from utils import gps2utc
from pymavlink import mavutil
import sys

print(sys.argv[2])
f=open(sys.argv[2], 'w')

print(sys.argv[1])
mavmaster = mavutil.mavlink_connection(sys.argv[1])

msg = mavmaster.recv_msg()
while msg is not None:
  if msg.get_type() == 'GPS':
    msg=msg.to_dict()
    f.write('{timeus}, {gpstime}\n'.format(timeus=msg['TimeUS']/1000000., gpstime=gps2utc(msg['GMS']/1000.,msg['GWk'])))
  msg = mavmaster.recv_msg()

f.close()
