#!/usr/bin/env python3

#### Parse /vnatt and /vnpos to a csv
# /vnatt is a sensor_msgs/Imu message, we only care about fields[2,4,5,6,7]
# /vnatt[2] = header.stamp
# /vnatt[4] = orientation.x
# /vnatt[5] = orientation.y
# /vnatt[6] = orientation.z
# /vnatt[7] = orientation.w
#
# /vnpos is a sensor_msgs/NavSatFix, we only care about fields[2,6,7,8]
# /vnpos[2] = header.stamp
# /vnpos[6] = lat
# /vnpos[7] = long
# /vnpos[8] = alt
####

import utm

def combine(att, pos, outdir):

  att_dict = {}
  pos_dict = {}

  attf = open(att)
  for line in attf:
    fields = line.strip().split(',')
    stamp = float(fields[2])
    q={}
    q['x'] = float(fields[4])
    q['y'] = float(fields[5])
    q['z'] = float(fields[6])
    q['w'] = float(fields[7])
    att_dict[stamp] = q
  attf.close()

  posf = open(pos)
  for line in posf:
    fields = line.strip().split(',')
    stamp = float(fields[2])
    p = {}
    p['lat'] = float(fields[6])
    p['lng'] = float(fields[7])
    p['alt'] = float(fields[8])
    pos_dict[stamp] = p
  posf.close()

  outpath = "%s/combined.csv" % outdir
  outf = open(outpath,'w')
  for stamp in att_dict:
    if stamp in pos_dict.keys():
      utm_pos = utm.from_latlon(pos_dict[stamp]['lat'], pos_dict[stamp]['lng'])
      line = ("%f,%f,%f,%f,%f,%f,%f,%f\n") % (stamp/10**9,utm_pos[1], utm_pos[0], -pos_dict[stamp]['alt'], att_dict[stamp]['x'],att_dict[stamp]['y'],att_dict[stamp]['z'],att_dict[stamp]['w'])
      outf.write(line)
  outf.close()


if __name__=='__main__':
  import sys
  if len(sys.argv) < 4:
    print("Not enough arguments. Usage: %s [att csv] [pos csv] [output csv dir]")
    print("run rostopic echo -b [file.bag] -p /[topic] > [output file] to generate each input file")
    sys.exit()
  att = sys.argv[1]
  pos = sys.argv[2]
  out = sys.argv[3]
  combine(att,pos, out)
