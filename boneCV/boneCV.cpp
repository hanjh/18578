/* boneCV.cpp
 *
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.derekmolloy.ie
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that source code redistributions retain this notice.
 *
 * This software is provided AS IS and it comes with no warranties of any type. 
 */

#include<iostream>
#include<opencv2/opencv.hpp>
#include <stdio.h>
#include<string.h>
#include <highgui.h>
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
/*
    VideoCapture capture(0);
    //capture.set(CV_CAP_PROP_FRAME_WIDTH,1920)
    //capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if(!capture.isOpened())
	    cout << "Failed to connect to the camera." << endl;
    cout << "Connected to the camera." << endl;
    */

    CvCapture* input_video = cvCreateCameraCapture(0); // video0
    Mat frame, edges;
    cvNamedWindow("window detection", CV_WINDOW_AUTOSIZE);

    //capture >> frame;

    //char* name = argv[1];
    
    // ignore capture
    /*
    frame = imread(name, CV_LOAD_IMAGE_COLOR);
    if(frame.empty())
    {
		cout << "Failed to capture an image" << endl;
		return -1;
    }
    */
    
    while(true)
    {
        const IplImage* video_frame = cvQueryFrame(input_video);
        if(video_frame == NULL)
        {
            printf("opening video failed\n");
            return -1;
        }
        // need to convert video_frame from IplImage* to Mat, because cvtColor 
        // takes mat
        Mat frame(video_frame);

        /* FILTERING */
        cvtColor(frame, edges, CV_BGR2GRAY);
        blur(edges, edges, Size(7, 7));
        Canny(edges, edges, 10, 50, 3);
            
        cvShowImage("window detection", edges);
    }

#if 0
    /* FIND CORNERS */
    // 640x480
    int mintop = edges.rows;
    int minleft = edges.cols;
    for (int i = 0; i < edges.rows; i++)
    {
        for (int j = 0; j < edges.cols; j++)
        {
            if (edges.at<uchar>(j, i) == 255)
            {
                if (i < mintop)
                {
                    mintop = i;
                }

                if (j < minleft)
                {
                    minleft = j;
                }
            }
        }
    }

    int maxbot = 0;
    int maxright = 0;
    for (int i = edges.rows; i > 0; i--)
    {
        for (int j = edges.cols; j > 0; j--)
        {
            if (edges.at<uchar>(j, i) == 255)
            {
                if (i > maxbot) 
                {
                    maxbot = i;
                }

                if (j > maxright)
                {
                    maxright = j;
                }
            }
        }
    }
    /*
    printf("minleft = %d\n", minleft);
    printf("mintop = %d\n", mintop);
    printf("maxright = %d\n", maxright);
    printf("maxbot = %d\n", maxbot);
    */
/*
    Point topleft = Point((double) minleft, (double) mintop);
    Point topright= Point((double) maxright, (double) mintop);
    Point botleft = Point((double) minleft, (double) maxbot);
    Point botright = Point((double) maxright, (double) maxbot);
    */
    Point topleft = Point((double) mintop, (double) minleft);
    Point topright= Point((double) mintop, (double) maxright);
    Point botleft = Point((double) maxbot, (double) minleft);
    Point botright = Point((double) maxbot, (double) maxright);

    line(edges, topleft, topright, Scalar(255), 10, 8, 0);
    line(edges, topleft, botleft, Scalar(255), 10, 8, 0);
    line(edges, topright, botright, Scalar(255), 10, 8, 0);
    line(edges, botleft, botright, Scalar(255), 10, 8, 0);
        /*
            if (i == j)
                edges.at<uchar>(j, i) = 255; // white
        */

    /* SAVING */
    char* edge_name = "edges.png";
    char* capture_name = "capture.png";
    if (!strcmp(argv[1], "capture-blinds.png"))
    {
        char* edge_name = "edges-blinds.png";
        //char* capture_name = "capture-blinds.png";
    }
    else
    {
        char* edge_name = "edges.png";
        //char* capture_name = "capture.png");
    }

    imwrite(edge_name, edges);
    //imwrite(capture_name, frame);

#endif
    return 0;
}
