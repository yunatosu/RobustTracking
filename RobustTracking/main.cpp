
//
//  main.cpp
//  RobustTracking
//
//  Created by Alper Yilmaz on 9/21/13.
//  Copyright (c) 2013 Alper Yilmaz. All rights reserved.
//


#include "cTrackingFramework.h"


//#include <core/core.hpp>
//#include <core/types_c.h>
//#include <video/tracking.hpp>
//#include <features2d/features2d.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    cTrackingFramework tracker;
    Mat frame,frame_smaller;
//    Mat gray;
    
//    VideoCapture cap(0);
//    VideoCapture cap("/Users/yilmaz.15/Dropbox/DaniyaPCV/calendar.mpg");
    VideoCapture cap("/Users/yilmaz.15/Dropbox/DaniyaPCV/IMG-0036.mov");
//    VideoCapture cap("/Users/yilmaz.15/Temporary/IMG_5350.MOV");
//    VideoCapture cap("/Users/yilmaz.15/Temporary/IMG_5362.MOV");

    if (!cap.isOpened())
    {
        // print error msg
        return -1;
    }
    
    namedWindow("Display window",WINDOW_AUTOSIZE);

    double image_resize_factor = 3.0;
    
    cap >> frame;
//    cvtColor(frame, gray, CV_BGR2GRAY);
    resize(frame,frame_smaller,Size(frame.cols/image_resize_factor,frame.rows/image_resize_factor),0,0,INTER_AREA);
    tracker.Initialize_Tracker(frame_smaller);
    
    char key;
    

    while (frame.data!=NULL) {// && tracker.processed_image_count<150) {
        //arrange image
        resize(frame,frame_smaller,Size(frame.cols/image_resize_factor,frame.rows/image_resize_factor),0,0,INTER_AREA);
        tracker.Insert_New_Image(frame_smaller);
        //perform tracking
        tracker.Compute_Motion();
        tracker.Display_Tracks(5);
        //keyboard analysis
        key = waitKey(10);
        switch (key) {
            case 's':
                key = 'c';
                while (key!='s') { key = waitKey(10); }
                break;
            case 'q':
                exit(1);
            default:
                break;
        }
        //read next frame
        cap >> frame;
    }
    
    return 0;
    
  /*
    
    
    Mat image;
    image = imread("/Users/yilmaz.15/Temporary/yos9.tif", 1);   // Read the file
    
    cTrackingFramework tracker;
    tracker.Initialize_Tracker(image);
    tracker.Display_Corners();
    
    image = imread("/Users/yilmaz.15/Temporary/yos10.tif", 1);   // Read the file
    tracker.Insert_New_Image(image);
    tracker.Display_Corners();

    tracker.Compute_Motion();
    tracker.Display_Tracks();

//    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
//    imshow( "Display window", image );                   // Show our image inside it.
//    waitKey(0);                                          // Wait for a keystroke in the window
    
    
    return 0;
   */
}





/*
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;

int maxCorners = 500;
int maxTrackbar = 100;

RNG rng(12345);
char* source_window = "Image";

/// Function header
void goodFeaturesToTrack_Demo( int, void* );

int main( int argc, char** argv )
{
    /// Load source image and convert it to gray
    src = imread( "/Users/yilmaz.15/Temporary/yos9.tif", 1 );
    cvtColor( src, src_gray, CV_BGR2GRAY );
    
    /// Create Window
    namedWindow( source_window, WINDOW_AUTOSIZE );
    
    /// Create Trackbar to set the number of corners
    createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, goodFeaturesToTrack_Demo );
    
    imshow( source_window, src );
    
    goodFeaturesToTrack_Demo( 0, 0 );
    
    waitKey(0);
    return(0);
}


void goodFeaturesToTrack_Demo( int, void* )
{
    if( maxCorners < 1 ) { maxCorners = 1; }
    
    /// Parameters for Shi-Tomasi algorithm
    vector<Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    
    /// Copy the source image
    Mat copy;
    copy = src.clone();
    
    /// Apply corner detection
    goodFeaturesToTrack( src_gray,
                        corners,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        useHarrisDetector,
                        k );
    
    
    /// Draw corners detected
    cout<<"** Number of corners detected: "<<corners.size()<<endl;
    int r = 4;
    for( int i = 0; i < corners.size(); i++ )
    { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255),
                                          rng.uniform(0,255)), -1, 8, 0 ); }
    
    /// Show what you got
    namedWindow( source_window, WINDOW_AUTOSIZE );
    imshow( source_window, copy );
}
*/

