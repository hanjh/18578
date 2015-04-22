#!/bin/bash

echo "Building..."
g++ window_detection.cpp -o wd -I /usr/include/opencv/ -lopencv_video -lopencv_core -lopencv_imgproc -lopencv_highgui
#g++ -O2 `pkg-config --cflags --libs opencv` window_detection.cpp -o window_detection
 
echo "Finished"
