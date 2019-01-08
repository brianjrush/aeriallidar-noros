#!/usr/bin/env python3

import struct
import json
from tools.utils import gps2utc
import sys

# format is qw qx qy qz tow gwk valid_flags
#
# valid_flags contains 3 bits of information
# Bit 0: PPS Beacon Good. 1 if GPS PPS is being used, 0 if AHRS internal clock
#   is providing the PPS.
# Bit 1: GPS Time Refresh. Asserted if we are regaining GPS signal after having
#   lost it. Usually accompanied by a jump in time.
# Bit 2: GPS Time Initialized. Set the first time we get time. Does not change 
#   even if we lose signal.

def parse_ahrs(infile, outfile):
  data_format="<ffffdHH"

  with open(infile, 'rb') as f:
    b = f.read()
    filesize = f.tell()

  unpacker = struct.Struct(data_format)
  data={}
  for message in unpacker.iter_unpack(b):
    flags = message[6]
    if flags >= 4:
      qw = message[0]
      qx = message[1]
      qy = message[2]
      qz = message[3]
      stamp = gps2utc(message[4], message[5])
      data[stamp] = {'qw': qw,
                     'qx': qx,
                     'qy': qy,
                     'qz': qz,
                     'flags': flags}
  with open(outfile, 'w') as f:
    json.dump(data, f, indent=2)




def parse_gps(infile, outfile):
  data_format="<ddddffdHH"

  with open(infile, 'rb') as f:
    b = f.read()
    filesize = f.tell()

  unpacker = struct.Struct(data_format)
  data={}
  for message in unpacker.iter_unpack(b):
    lat = message[0]
    lng = message[1]
    ellipsoid_height = message[2]
    msl_height = message[3]
    horiz_accuracy = message[4]
    vert_accuracy = message[5]
    stamp = gps2utc(message[6], message[7])
    flags = message[8]
    data[stamp] = {'latitude': lat,
                   'longitute': lng,
                   'ellipsoid_height': ellipsoid_height,
                   'msl_height': msl_height,
                   'horizontal_accuracy': horiz_accuracy,
                   'vertical_accuracy': vert_accuracy,
                   'flags': flags}
  with open(outfile, 'w') as f:
    json.dump(data, f, indent=2)

if __name__ == '__main__':
  parse_ahrs("ahrs.bin", "ahrs.json")
  parse_gps("gps.bin", "gps.json")
