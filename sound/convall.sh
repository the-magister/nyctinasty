#!/bin/bash

#usage conv.sh startcount

#idle = 100
#intro = 200
#win = 300
#c1 = 400
#c2 = 500
#c3 = 600

rm -rf converted/*

let start=100;
let cnt=0;

for filename in idle/*.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

let start=200;
let cnt=0;

for filename in intro/*.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

let start=300;
let cnt=0;

for filename in win/*.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

let start=400;
let cnt=0;

for filename in c1/*.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

let start=500;
let cnt=0;

for filename in c2/*.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

let start=600;
let cnt=0;

for filename in c3/*.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done

