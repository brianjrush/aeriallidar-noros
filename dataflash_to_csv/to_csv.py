#!/usr/bin/env python3

from pymavlink import mavutil

import sys
mavmaster = mavutil.mavlink_connection(sys.argv[1])

files = {}
mavmsg = mavmaster.recv_msg()

while mavmsg is not None:
  msg_type = mavmsg.get_type()
  print("Reading %s message" % msg_type)
  if msg_type == 'NKF1':
    # PD
    if 'NKF1' not in files.keys():
      print('Opening nkf1-pd.csv')
      files['NKF1'] = open('nkf1-pd.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['PD'])
    f = files['NKF1']
    f.write(line)
  elif msg_type == 'GPS':
    # Alt
    if 'GPS' not in files.keys():
      print('Opening gps-alt.csv')
      files['GPS'] = open('gps-alt.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['Alt'])
    f = files['GPS']
    f.write(line)
  elif msg_type == 'GPS2':
    # Alt
    if 'GPS2' not in files.keys():
      print('Opening gps2-alt.csv')
      files['GPS2'] = open('gps2-alt.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['Alt'])
    f = files['GPS2']
    f.write(line)
  elif msg_type == 'CTUN':
    # DAlt, Alt, BAlt, DSAlt, SAlt, TAlt
    if 'CTUN' not in files.keys():
      files['CTUN'] = {}
      if 'DAlt' not in files['CTUN'].keys():
        print('Opening ctun-dalt.csv')
        files['CTUN']['DAlt'] = open('ctun-dalt.csv', 'w')
      if 'Alt' not in files['CTUN'].keys():
        print('Opening ctun-alt.csv')
        files['CTUN']['Alt'] = open('ctun-alt.csv', 'w')
      if 'BAlt' not in files['CTUN'].keys():
        print('Opening ctun-balt.csv')
        files['CTUN']['BAlt'] = open('ctun-balt.csv', 'w')
      if 'DSAlt' not in files['CTUN'].keys():
        print('Opening ctun-dsalt.csv')
        files['CTUN']['DSAlt'] = open('ctun-dsalt.csv', 'w')
      if 'SAlt' not in files['CTUN'].keys():
        print('Opening ctun-salt.csv')
        files['CTUN']['SAlt'] = open('ctun-salt.csv', 'w')
      if 'TAlt' not in files['CTUN'].keys():
        print('Opening ctun-talt.csv')
        files['CTUN']['TAlt'] = open('ctun-talt.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['DAlt'])
    f = files['CTUN']['DAlt']
    f.write(line)
    line = '%d,%f\n' % (msg['TimeUS'], msg['Alt'])
    f = files['CTUN']['Alt']
    f.write(line)
    line = '%d,%f\n' % (msg['TimeUS'], msg['BAlt'])
    f = files['CTUN']['BAlt']
    f.write(line)
    line = '%d,%f\n' % (msg['TimeUS'], msg['DSAlt'])
    f = files['CTUN']['DSAlt']
    f.write(line)
    line = '%d,%f\n' % (msg['TimeUS'], msg['SAlt'])
    f = files['CTUN']['SAlt']
    f.write(line)
    line = '%d,%f\n' % (msg['TimeUS'], msg['TAlt'])
    f = files['CTUN']['TAlt']
    f.write(line)

  elif msg_type == 'BAR2':
    # Alt
    if 'BAR2' not in files.keys():
      print('Opening bar2-alt.csv')
      files['BAR2'] = open('bar2-alt.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['Alt'])
    f = files['BAR2']
    f.write(line)

  elif msg_type == 'BARO':
    # Alt
    if 'BARO' not in files.keys():
      print('Opening baro-alt.csv')
      files['BARO'] = open('baro-alt.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['Alt'])
    f = files['BARO']
    f.write(line)

  elif msg_type == 'AHR2':
    # Alt
    if 'AHR2' not in files.keys():
      print('Opening ahr2-alt.csv')
      files['AHR2'] = open('ahr2-alt.csv', 'w')
    msg = mavmsg.to_dict()
    line = '%d,%f\n' % (msg['TimeUS'], msg['Alt'])
    f = files['AHR2']
    f.write(line)
  mavmsg = mavmaster.recv_msg()

if 'NKF1' in files.keys():
  files['NKF1'].close()
if 'GPS' in files.keys():
  files['GPS'].close()
if 'GPS2' in files.keys():
  files['GPS2'].close()
if 'CTUN' in files.keys():
  files['CTUN']['DAlt'].close()
  files['CTUN']['Alt'].close()
  files['CTUN']['BAlt'].close()
  files['CTUN']['DSAlt'].close()
  files['CTUN']['SAlt'].close()
  files['CTUN']['TAlt'].close()
if 'BARO' in files.keys():
  files['BARO'].close()
if 'BAR2' in files.keys():
  files['BAR2'].close()
if 'AHR2' in files.keys():
  files['AHR2'].close()
