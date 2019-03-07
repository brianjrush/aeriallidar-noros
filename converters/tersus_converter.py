#!/usr/bin/env python3

from datetime import datetime
import json
import sys
import pytz

def parse_line(line):
  pattern = '%Y/%m/%d %H:%M:%S.%f'
  data = {}
  line = ' '.join(line.split()).split()
  
  date_string = line[0] + ' ' + line[1]
  stamp = datetime.strptime(date_string, pattern)
  stamp = stamp.replace(tzinfo=pytz.UTC).timestamp()

  data['lat'] = float(line[2])
  data['long'] = float(line[3])
  data['height'] = float(line[4])
  data['Q'] = int(line[5])
  data['ns'] = int(line[6])
  return {float(stamp):data}

if __name__ == '__main__':
  data={}
  with open(sys.argv[1]) as f:
    for line in f:
      if line[0] == '%':
        continue
      data.update(parse_line(line))
  with open(sys.argv[2], 'w') as f:
    json.dump(data, f, indent=2)
