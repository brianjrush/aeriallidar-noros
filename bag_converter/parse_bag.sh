#!/bin/bash

if [ -z "$1" ]; then
	echo "Please supply the path to a bag file."
	exit
else
	bag="$1"
fi
if [ -z "$2" ]; then
	dir="input"
else
	dir="$2"
fi
echo "Reading from /Sensor/points. This will take some time"
mkdir -p "$dir"
python2 bag_to_ply.py "$bag" "$dir"
echo -n "Reading /vnatt from $bag... "
rostopic echo -b "$bag" -p /vnatt > orientation
echo "done."
echo -n "Reading /vnpos from $bag... "
rostopic echo -b "$bag" -p /vnpos > position
echo "done."

tail -n +2 orientation > orientation.csv
tail -n +2 position > position.csv
rm orientation
rm position

echo "Combining files..."
python3 combine_csv.py orientation.csv position.csv "$dir"
rm orientation.csv
rm position.csv
