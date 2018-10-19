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

data_format="<ffffdHH"

with open(sys.argv[1], 'rb') as f:
  b = f.read()
  filesize = f.tell()

unpacker = struct.Struct(data_format)
data={}
for message in unpacker.iter_unpack(b):
  qw = message[0]
  qx = message[1]
  qy = message[2]
  qz = message[3]
  stamp = gps2utc(message[4], message[5])
  flags = message[6]
  data[stamp] = {'qw': qw,
                 'qx': qx,
                 'qy': qy,
                 'qz': qz,
                 'flags': flags}
with open(sys.argv[2], 'w') as f:
  json.dump(data, f)
