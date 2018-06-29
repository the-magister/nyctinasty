#!/bin/bash

#usage conv.sh startcount

#idle = 100
#intro = 200
#win = 300

let start=$1;
let cnt=0;
for filename in *.wav; do
   ((num=$start + $cnt));
   echo cnt: "$cnt $filename $num.wav";
    sox "$filename" -t wav -b 16 "../converted/$num.wav" rate 44100 channels 2 norm -1 silence 1 1 0.01%
   ((cnt++));
done
