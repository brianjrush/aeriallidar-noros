#!/bin/bash
set -e
OPTIND=1
TMPDIR="/tmp/lidar"

usage() {
  echo "Usage: `basename "$0"` [options] dataflash_log [cloud1.ply cloud2.ply...]"
  echo " -h  display this message"
  echo " -t  set the temp directory in RAM"
}

while getopts "ht:" OPTION; do
  case "$OPTION" in
    h) usage;;
	t) TMDIR="$OPTARG";;
	\?) echo "Unknown option -$OPTARG"; exit 1;;
	:) echo "Missing option argument of -$OPTARG"; exit 1;;
  esac
done
shift $(($OPTIND-1))
if [ "$#" -lt 1 ]; then echo "Not enough arguments"; usage; exit 1; fi
dataflash_log="$1"
shift 1

mkdir -p "$TMPDIR"

json="$TMPDIR/tf.json"
sectionjson="$TMPDIR/sections.json"
./dataflash_converter.py "$dataflash_log" "$json" "$sectionjson"

echo "Loading clouds into RAM..."
mkdir -p "$TMPDIR/clouds"
count=0
for file in "$@"; do
cp "$file" "$TMPDIR/clouds/"
if [ $(($count%500)) -eq 0 ]; then echo "$count/$# files moved"; fi
count=$(($count+1))
done
echo "$count/$# files moved"
echo "Done"
