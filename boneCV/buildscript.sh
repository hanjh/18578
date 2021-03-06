#!/bin/bash
 
echo "Video for Beaglebone Video Applications - derekmolloy.ie"
 
echo "Building the OpenCV example for the Beaglebone"
g++ -O2 `pkg-config --cflags --libs opencv` boneCV.cpp -o boneCV
 
echo "Building the OpenCV timing example for the Beaglebone"
g++ -O2 `pkg-config --cflags --libs opencv` -lrt boneCVtiming.cpp -o boneCVtiming
 
echo "Building the Video4Linux frame capture program"
gcc -O2 -Wall `pkg-config --cflags --libs libv4l2` grabber.c -o grabber
 
echo "Building the Video4Linux capture example program"
gcc -O2 -Wall `pkg-config --cflags --libs libv4l2` capture.c -o capture
 
echo "Finished"
