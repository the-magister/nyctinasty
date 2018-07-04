#!/bin/bash

#usage conv.sh startcount

#idle = 100
#intro = 200
#win = 300
#c1 = 400
#c2 = 500
#c3 = 600
#beat = 700
#poschange = 800
#negchange = 900
#gunfire = 950

rm -rf converted/*

let start=100;
let cnt=0;

for filename in idle/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "lonely count: $cnt";

let start=200;
let cnt=0;

for filename in intro/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "ohai num: $cnt";

let start=300;
let cnt=0;

for filename in win/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "win num: $cnt"

let start=400;
let cnt=0;

for filename in c1/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "goodnuf cnt: $cnt";

let start=500;
let cnt=0;

for filename in c2/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "goodjob cnt: $cnt";

let start=600;
let cnt=0;

for filename in c3/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

echo "winning cnt: $cnt";

let start=700;
let cnt=0;

for filename in beat/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

echo "beat cnt: $cnt";

let start=800;
let cnt=0;

for filename in poschange/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "poschange cnt: $cnt";

let start=900;
let cnt=0;

for filename in negchange/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

echo "negchange cnt: $cnt";

let start=950;
let cnt=0;

for filename in gunfire/*.wav; do
   ((num=$start + $cnt));
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
echo "gunfire cnt: $cnt"
