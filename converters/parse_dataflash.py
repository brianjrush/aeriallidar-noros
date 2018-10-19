#!/usr/bin/env python3

from tools import utils
import numpy as np
import json
import utm
import sys

with open(sys.argv[1]) as f:
  data=json.load(f)

msg_types = ['NKQ1', 'NKF1'] #['NKF1', 'NKQ1', 'BARO']
lengths = [len(data[fmt].keys()) for fmt in msg_types]
slowest_msg_idx = np.argmin(lengths)
slowest_msg = msg_types[slowest_msg_idx]

stamp_arrays = {}
for msg in msg_types:
  stamp_arrays[msg] = np.array([float(stamp) for stamp in data[msg].keys()])

out = {}
for stamp in data[slowest_msg].keys():
  stamp = float(stamp)
  idx, err = utils.get_closest_stamp(stamp, stamp_arrays['NKF1'])
  nkf1 = data['NKF1'][str(stamp_arrays['NKF1'][idx])]
  #idx, err = self.get_closest_stamp(stamp, stamp_arrays['BARO'])
  #baro = data['BARO'][str(stamp_arrays['BARO'][idx])]
  idx, err = utils.get_closest_stamp(stamp, stamp_arrays['NKQ1'])
  nkq1 = data['NKQ1'][str(stamp_arrays['NKQ1'][idx])]
  #idx, err = self.get_closest_stamp(stamp, stamp_arrays['POS'])
  #pos = data['POS'][str(stamp_arrays['POS'][idx])]
  # ['NKQ1']['Q1'] is 'w' ['Q2']->['Q4'] are XYZ
  ## https://github.com/ArduPilot/ardupilot/blob/20d22f3629d81ccd69282c38903b46434c980ef5/libraries/AP_Math/quaternion.h
  
  #utminfo = utm.from_latlon(pos['Lat'], pos['Lng'])
  out[stamp] = {'x':nkf1['PE'], 'y':nkf1['PN'], 'z':-nkf1['PD'], 'qx':nkq1['Q2'], 'qy':nkq1['Q3'], 'qz':nkq1['Q4'], 'qw':nkq1['Q1']}

with open(sys.argv[2], 'w') as f:
  json.dump(out, f)
