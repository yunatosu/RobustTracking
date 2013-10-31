//
//  cTrackingFramework.h
//  RobustTracking
//
//  Created by Alper Yilmaz on 9/21/13.
//  Copyright (c) 2013 Alper Yilmaz. All rights reserved.
//

#ifndef __RobustTracking__cTrackingFramework__
#define __RobustTracking__cTrackingFramework__

#include "globals.h"

class cTrackingFramework{

private:
//    bool buffer_allocated;
    void allocate_image_buffer();
    bool geometry_initialized;
    RNG  rng;
    Mat  DCT_space;
    Mat  DCT_space_right_side;
    int  DCT_K_parameter;
    
    Mat    Estimate_Camera(vector <Mat> &structure,vector<Point2f> &uv);
    Mat    Estimate_Camera(vector<int> &indices);
    double Test_Camera(Mat &camera_DCT,Mat &DCT_coefficients,Point2f &prev,Point2f &curr);
    void   Tracking_Post_Process();
//    void   Add_New_Points(long count);
    void   Add_New_Points();
    void   Estimate_Structure(vector <int> &tracklets_for_structure_estimation);
    void   Initialize_DCT_Space();
public:
    cTrackingFramework();
    cTrackingFramework(Mat &first_image);//no need to call init_tracker is this called first
    ~cTrackingFramework();

    int                     processed_image_count; //holds the number of total images processed
    vector<VIDEO_FRAME>     image_buffer;
    vector<TRACKLET_RECORD> tracklet_record;
    void Initialize_Tracker(Mat &first_image); // this has to be called once before traking stars. if you used the constructor which sends in image as well you can skip this function
    void Insert_New_Image(Mat &new_image);
    void Compute_Motion(); //called with each inserted image
    void Subspace_Geometry_Initialize_From_Motion();
    vector<int> Generate_Inliers_For_New_Frame_RANSAC();
    
    void Display_Tracks(int tail_length);
};

#endif /* defined(__RobustTracking__cTrackingFramework__) */















