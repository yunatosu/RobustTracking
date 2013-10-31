//
//  globals.h
//  RobustTracking
//
//  Created by Alper Yilmaz on 9/24/13.
//  Copyright (c) 2013 Alper Yilmaz. All rights reserved.
//

#ifndef RobustTracking_globals_h
#define RobustTracking_globals_h

#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#define __ESTIMATE__STRUCTURE__ONCE__
//#undef  __ESTIMATE__STRUCTURE__ONCE__

#define __WEIGHTED__LEAST__SQUARES__  1

#define __COUT__LEVEL__1__
#define __COUT__LEVEL__2__
//#define __COUT__IMAGE__INDICES__


#define __MAX__CORNERS__             500

#define __SUBSPACE__DIMENSION__      6
#define __DCT__K__PARAMETER__        1

#define __MAX__TRACKLET_RECORD__LENGTH__    1000
#define __MAX_ALLOWED__OCCLUSION__FRAMES__BEFORE__DELETION__ 90
#define __TEMPLATE__SIZE__                  15

#define __BORDER__LIMIT__                   15

#define __RANSAC__SAMPLE__SIZE__            8
#define __RANSAC__MAX__TRIAL__COUNT__       90

#define __TRACKING__PYRAMID__LEVEL__        1

#define __THRESHOLD__RANSAC__DISTANCE__     4.0 //should be sqr of regular distance
#define __THRESHOLD__TRACKING__MIN__EIG__   0.01
#define __THRESHOLD__TRACKING__ERROR__      0.70
#define __THRESHOLD__INLIER__SET__          75 //percentage
#define __THRESHOLD__REPROJECTION__ERROR__  5
#define __THRESHOLD__OCCLUSION__RECOVERY__  50 //square of appearance similarity

#define __FLAG__NONE__                   0
#define __FLAG__OCCLUDED__               1
#define __FLAG__INITIALIZED__            2 //STRUCTURE INITIALZED
#define __FLAG__OCCLUDED__INITIALIZED__  3 //__FLAG__OCCLUDED__+__FLAG__OCCLUDED__INITIALIZED__

// image buffer size has to be bigger than # of frames required to compute structure !!
#define __IMAGE__BUFFER__SIZE__          30
#define __FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__ 30


using namespace std;
using namespace cv;

struct VIDEO_FRAME {
    vector<Point2f> corners;
    Mat   gray_image;
    Mat   color_image;
    Mat   camera; // 6x1 alpha beta gamma tx ty tz COMPUTED FROM SVD
};

struct MY_POINT {
    Point2f position;
    double  tracking_error;
};

struct TRACKLET_RECORD {
    vector<MY_POINT> point;
    int starting_frame_no;
    int life; //number of frames observed is used in computing gemoetry and checking validity of geo_location
    uchar flag;
    int  occlusion_frame_count;
    Mat DCT_coefficients;
    Scalar_<int> color_code;
    Mat template_before_occlusion;
};



#endif
