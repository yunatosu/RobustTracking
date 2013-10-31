//
//  cTrackingFramework.cpp
//  RobustTracking
//
//  Created by Alper Yilmaz on 9/21/13.
//  Copyright (c) 2013 Alper Yilmaz. All rights reserved.
//

#include "cTrackingFramework.h"


cTrackingFramework::cTrackingFramework() {
    rng = RNG(0xFFFFFFFF);
    if (__IMAGE__BUFFER__SIZE__<__FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__) {
        cout<<"__FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__ cannot be smaller than __IMAGE__BUFFER__SIZE__"<<endl;
        exit(-1);
    }
    allocate_image_buffer();
    processed_image_count=0;
    Initialize_DCT_Space();
}

cTrackingFramework::cTrackingFramework(Mat &first_image) {
    rng = RNG(0xFFFFFFFF);
    if (__IMAGE__BUFFER__SIZE__<__FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__) {
        cout<<"__FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__ cannot be smaller than __IMAGE__BUFFER__SIZE__"<<endl;
        exit(-1);
    }
    allocate_image_buffer();
    Initialize_Tracker(first_image);
    Initialize_DCT_Space();
}

cTrackingFramework::~cTrackingFramework() {
}

void cTrackingFramework::allocate_image_buffer() {
    processed_image_count=0;
    image_buffer = vector<VIDEO_FRAME>(__IMAGE__BUFFER__SIZE__);
    for(int i=0;i<__IMAGE__BUFFER__SIZE__;i++){
        image_buffer.at(i).corners=vector<Point2f>(__MAX__CORNERS__);
    }
}

void cTrackingFramework::Initialize_DCT_Space() {
    DCT_K_parameter = __DCT__K__PARAMETER__;
    double N =__SUBSPACE__DIMENSION__*DCT_K_parameter;
    DCT_space = Mat(__SUBSPACE__DIMENSION__,N,CV_64F);
    for (int k=0;k<__SUBSPACE__DIMENSION__;k++){
        for (int n=0;n<N;n++){
            double val= cos( (M_PI/N) * ( (double)n + 0.5 ) * (double)k );
            DCT_space.at<double>(k,n) = val;
        }
    }
    DCT_space_right_side = (DCT_space.t()*DCT_space).inv()*DCT_space.t();
}

void cTrackingFramework::Insert_New_Image(Mat &new_image) {
    int index;
    //allocate image variable
    index = processed_image_count % __IMAGE__BUFFER__SIZE__;
    processed_image_count ++;
    image_buffer.at(index).color_image = new_image.clone();
    cvtColor(image_buffer.at(index).color_image, image_buffer.at(index).gray_image, CV_BGR2GRAY);
    //compute features for all coming images to add corners if some are occluded
    goodFeaturesToTrack(image_buffer.at(index).gray_image,
                        image_buffer.at(index).corners,
                        2*__MAX__CORNERS__,
                        0.01 /*double qualityLevel*/,
                        2 /*double minDistance*/);//,InputArray mask=noArray(), int blockSize=3, bool useHarrisDetector=false, double k=0.04 )
    cornerSubPix(image_buffer.at(index).gray_image,
                 image_buffer.at(index).corners,
                 Size(__TEMPLATE__SIZE__/2,__TEMPLATE__SIZE__/2),
                 Size(-1,-1),
                 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
    image_buffer.at(index).camera = Mat(2,__SUBSPACE__DIMENSION__,CV_64F,0.0);
}

void cTrackingFramework::Initialize_Tracker(Mat &first_image){
    TRACKLET_RECORD new_trajectory;
    MY_POINT my_point;
    new_trajectory.flag = __FLAG__NONE__;
    new_trajectory.occlusion_frame_count = 0;
    new_trajectory.starting_frame_no=0;
    new_trajectory.life=1;
    my_point.tracking_error = 0;

    Insert_New_Image(first_image);
    
//    Add_New_Points(__MAX__CORNERS__);
    Add_New_Points();
}

void cTrackingFramework::Display_Tracks(int tail_length) {
    int i,curr_image_index;
    Mat temp_image;
    
    curr_image_index = (__IMAGE__BUFFER__SIZE__+processed_image_count-1) % __IMAGE__BUFFER__SIZE__;
    Mat camera_DCT = image_buffer.at(curr_image_index).camera*DCT_space;
    
    temp_image = image_buffer.at(curr_image_index).color_image.clone();
    for(i = 0; i < tracklet_record.size(); i++ ){
        Point2f x_curr =tracklet_record.at(i).point.back().position;
        int possible_lenght = max ( 0,tracklet_record.at(i).life - tail_length);
        possible_lenght = tracklet_record.at(i).life - possible_lenght;
        
#ifdef __COUT__IMAGE__INDICES__
        Point2f xText =x_curr; xText.x -= 10; xText.y -= 4;
        ostringstream convert; convert << i;
        putText(temp_image, convert.str(), xText, FONT_HERSHEY_PLAIN, .8, Scalar(0,0,0), 2.0, 8, false );
        putText(temp_image, convert.str(), xText, FONT_HERSHEY_PLAIN, .8, Scalar(255,255,255), 1.3, 8, false );
        //                line(temp_image,x_curr,x_prev,Scalar(255, 255, 255),1.5);
#endif
        if ((tracklet_record.at(i).flag & __FLAG__OCCLUDED__)==0) { //if not occluded
            if ( (tracklet_record.at(i).flag & __FLAG__INITIALIZED__) !=0 ) {
                int index_start = max(0, tracklet_record.at(i).life - 1 - tail_length);
                int index_end   = tracklet_record.at(i).life-2;
                
                for (int j=index_start;j<=index_end;j++) {
                    line(temp_image,
                         tracklet_record.at(i).point.at(j  ).position,
                         tracklet_record.at(i).point.at(j+1).position,
                         Scalar(255, 255, 255),
                         2);
                }
                circle( temp_image,x_curr,4,tracklet_record.at(i).color_code,1);
                Point2f x_prev =tracklet_record.at(i).point.at(tracklet_record.at(i).life-2).position;
                //draw projected location
                Mat uv = camera_DCT*tracklet_record.at(i).DCT_coefficients;
                Point2f prjpnt;
                prjpnt.x = uv.at<double>(0)+x_prev.x;
                prjpnt.y = uv.at<double>(1)+x_prev.y;
                circle( temp_image,prjpnt,3, Scalar(255,255,255),-1);
                circle( temp_image,prjpnt,2, tracklet_record.at(i).color_code,-1);

            }
        }
        else {
            if (tracklet_record.at(i).life>1) {
                if ( (tracklet_record.at(i).flag & __FLAG__INITIALIZED__) !=0 ) {
                    rectangle( temp_image,x_curr-Point2f(2.,2.),x_curr+Point2f(2.,2.),
                              tracklet_record.at(i).color_code,-1);
                }
            }
        }
    }
    ostringstream window_name;
    window_name << processed_image_count;
    putText(temp_image, window_name.str(), Point2f(10,20), FONT_HERSHEY_PLAIN, .8, Scalar(255,0,0), 3.0, 8, false );
    putText(temp_image, window_name.str(), Point2f(10,20), FONT_HERSHEY_PLAIN, .8, Scalar(0,255,255),1.5,8, false );

    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", temp_image );                   // Show our image inside it.
    //waitKey(0);                                          // Wait for a keystroke in the window
}

void cTrackingFramework::Compute_Motion() {
    int curr_image_index,prev_image_index;
    vector<uchar>    status;
    vector<float>   error;
    
    curr_image_index = (__IMAGE__BUFFER__SIZE__+processed_image_count-1) % __IMAGE__BUFFER__SIZE__;
    prev_image_index = (__IMAGE__BUFFER__SIZE__+processed_image_count-2) % __IMAGE__BUFFER__SIZE__;
    
    int rows =image_buffer.at(curr_image_index).gray_image.rows;
    int cols =image_buffer.at(curr_image_index).gray_image.cols;
    
    MY_POINT        new_point;
    
    cout <<endl<< " + Processed image count:"<<processed_image_count<<"  ";
    
    
    vector<Point2f> previous_corners;
    vector<int>     previous_corner_indices;
    vector<Point2f> current_corners;

    for (int i=0;i<tracklet_record.size();i++) {
        if ( (tracklet_record.at(i).flag & __FLAG__OCCLUDED__)==0 ) {
            previous_corners.push_back(tracklet_record.at(i).point.back().position);
            previous_corner_indices.push_back(i);
        }
        else {
            new_point.tracking_error = 100;
            new_point.position = Point2f(-1,-1);
            tracklet_record.at(i).occlusion_frame_count++;
            tracklet_record.at(i).life++;
            tracklet_record.at(i).point.push_back(new_point);
        }
    }
    calcOpticalFlowPyrLK(image_buffer.at(prev_image_index).gray_image,
                         image_buffer.at(curr_image_index).gray_image,
                         previous_corners,current_corners,status,error,
                         Size(__TEMPLATE__SIZE__,__TEMPLATE__SIZE__),
                         __TRACKING__PYRAMID__LEVEL__,
                         cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.1),
                         OPTFLOW_LK_GET_MIN_EIGENVALS,
                         __THRESHOLD__TRACKING__MIN__EIG__ );
    for (int i=0;i<previous_corners.size();i++) {
        int index = previous_corner_indices.at(i);
        new_point.tracking_error = 100;
        new_point.position = Point2f(-1,-1);
        
        float controlx = (current_corners.at(i).x-__BORDER__LIMIT__)*
                         (cols-__BORDER__LIMIT__-current_corners.at(i).x);
        float controly = (current_corners.at(i).y-__BORDER__LIMIT__)*
                         (rows-__BORDER__LIMIT__-current_corners.at(i).y);
        bool control   = min(controlx,controly) >0 ;
        if (status.at(i) && control && error.at(i)<__THRESHOLD__TRACKING__ERROR__) {
            new_point.tracking_error = error.at(i);
            new_point.position = current_corners.at(i);
            try {
                tracklet_record.at(index).template_before_occlusion =image_buffer.at(prev_image_index).gray_image(cv::Rect(previous_corners.at(i).x-10,previous_corners.at(i).y-10,21,21));
            } catch (...) {
                cout << " - point " << index << " too close to border" << endl;
            }
        }
        else {
            //                cout << " - Tracking error " << error.at(0) << endl;
            tracklet_record.at(index).flag |= __FLAG__OCCLUDED__;
            tracklet_record.at(index).occlusion_frame_count=1;
        }
        tracklet_record.at(index).life++;
        tracklet_record.at(index).point.push_back(new_point);
    }

    
    
    
    
    
    
//    vector<Point2f> previous_corner;
//    vector<Point2f> current_corner;
//    for (int i=0;i<tracklet_record.size();i++) {
//        new_point.tracking_error = 100;
//        new_point.position = Point2f(-1,-1);
//        TRACKLET_RECORD *record = &tracklet_record.at(i);
//        if ( ((*record).flag & __FLAG__OCCLUDED__)==0 ) {
//            previous_corner = vector<Point2f>(1);
//            previous_corner.at(0) = (*record).point.at((*record).life-1).position;
//            calcOpticalFlowPyrLK(image_buffer.at(prev_image_index).gray_image,
//                                 image_buffer.at(curr_image_index).gray_image,
//                                 previous_corner,current_corner,status,error,
//                                 Size(__TEMPLATE__SIZE__,__TEMPLATE__SIZE__),
//                                 __TRACKING__PYRAMID__LEVEL__,
//                                 cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.1),
//                                 OPTFLOW_LK_GET_MIN_EIGENVALS,
//                                 __THRESHOLD__TRACKING__MIN__EIG__ );
//            float controlx = (current_corner.at(0).x-__BORDER__LIMIT__)*(image_buffer.at(0).gray_image.cols-__BORDER__LIMIT__-current_corner.at(0).x);
//            float controly = (current_corner.at(0).y-__BORDER__LIMIT__)*(image_buffer.at(0).gray_image.rows-__BORDER__LIMIT__-current_corner.at(0).y);
//            bool control   = min(controlx,controly) >0 ;
//            if (status.at(0) && control && error.at(0)<__THRESHOLD__TRACKING__ERROR__) {
//                new_point.tracking_error = error.at(0);
//                new_point.position = current_corner.at(0);
//                try {
//                    (*record).template_before_occlusion =image_buffer.at(prev_image_index).gray_image(cv::Rect(previous_corner.at(0).x-10,previous_corner.at(0).y-10,21,21));
//                } catch (...) {
//                    cout << " - point " << i << " too close to border" << endl;
//                }
//            }
//            else {
////                cout << " - Tracking error " << error.at(0) << endl;
//                (*record).flag |= __FLAG__OCCLUDED__;
//                (*record).occlusion_frame_count=1;
//            }
//        }
//        else {
//            (*record).occlusion_frame_count++;
//        }
//        (*record).life++;
//        (*record).point.push_back(new_point);
//    }
    Tracking_Post_Process();
}

void cTrackingFramework::Tracking_Post_Process() {
    long initial_tracklet_count,final_tracklet_count;
    int curr_image_index;
    vector <int> tracklets_for_structure_estimation;
    Mat camera_DCT;

    if (processed_image_count > __IMAGE__BUFFER__SIZE__) {
        curr_image_index = (__IMAGE__BUFFER__SIZE__+processed_image_count-1) % __IMAGE__BUFFER__SIZE__;
//        cout <<endl<<endl<<camera_DCT<<endl<<endl<<image_buffer.at(curr_image_index).camera<<endl<<endl<<DCT_space<<endl<<endl;
        
        initial_tracklet_count =  tracklet_record.size();
        
        //-----------------------------------------------------------------------------------------------
        // estimate camera geometry and
        // see there are additional occluded points due to motions that do not satisfy geometry
        vector <int> inlier_set_indices = Generate_Inliers_For_New_Frame_RANSAC();
        
        //recompute the camera matrix from the inlier set
        if (inlier_set_indices.size()>3) {
            vector <bool> inlier_control(tracklet_record.size(),false);
            image_buffer.at(curr_image_index).camera = Estimate_Camera(inlier_set_indices);
            camera_DCT = image_buffer.at(curr_image_index).camera*DCT_space;
//            cout << "Cam"<<curr_image_index<<" = "<<image_buffer.at(curr_image_index).camera<<endl<<endl;
#ifdef __COUT__LEVEL__2__
            cout <<" - Inlier set size: "<<inlier_set_indices.size()<<" ("<<(double)inlier_set_indices.size()/(double)tracklet_record.size()*100.0<<"%)"<<endl;
#endif
            // every point that out of inlier set is considered occluded
            for (int i=0;i<inlier_set_indices.size();i++) {
                inlier_control.at(inlier_set_indices.at(i))=true;
            }
            for (int i=0;i<tracklet_record.size();i++) {
                TRACKLET_RECORD *record = &tracklet_record.at(i);
                if (!inlier_control.at(i) && (*record).flag == __FLAG__INITIALIZED__) {
                    (*record).flag |= __FLAG__OCCLUDED__;
                    (*record).occlusion_frame_count++;
                }
            }
        }
        else {
            cout << " - NO INLIERS"<<endl<< processed_image_count<< " images have been processed"<<endl;
            exit(-1);
        }
        //-----------------------------------------------------------------------------------------------
        //perform pointwise operations
        double inlier_ratio = (double)inlier_set_indices.size()/(double)tracklet_record.size();
#ifdef __COUT__LEVEL__1__
        cout << " - Inlier ratio "<<inlier_ratio;
#endif
        for (int i = 0 ; i < tracklet_record.size();i++){
            TRACKLET_RECORD *record = &tracklet_record.at(i);
            if ((*record).flag == __FLAG__OCCLUDED__INITIALIZED__) {
                if ((*record).occlusion_frame_count>__MAX_ALLOWED__OCCLUSION__FRAMES__BEFORE__DELETION__) {
#ifdef __COUT__LEVEL__2__
                    cout << " - Erased "<<i<<endl;
#endif
                    tracklet_record.erase(tracklet_record.begin()+i);
                    i--; //since erased we need to subtract 1 from i
                }
                else {
                    //find reprojected locations
                    vector <MY_POINT> *point = &((*record).point);
                    Point2f x_prev =(*point).at((*record).life-2).position;
                    Mat uv = camera_DCT*(*record).DCT_coefficients;
//                    cout <<uv<<endl<<endl<<camera_DCT<<endl<<endl;
                    (*point).back().position.x = uv.at<double>(0)+x_prev.x;
                    (*point).back().position.y = uv.at<double>(1)+x_prev.y;
                    (*record).occlusion_frame_count++;

                    //check if there is similarity is to old template and make point unoccluded again
                    try {
                        Mat difference = (*record).template_before_occlusion - image_buffer.at(curr_image_index).gray_image(cv::Rect((*point).back().position.x-10,(*point).back().position.y-10,21,21));
                        //cout<<difference;
                        double ssd=0;
                        for (int j=0;j<difference.rows;j++) {
                            for (int k=0;k<difference.cols;k++) {
                                ssd += difference.at<char>(j,k)*difference.at<char>(j,k);
                            }
                        }
                        ssd = ssd/441.0;
                        if (ssd < __THRESHOLD__OCCLUSION__RECOVERY__) {
                            (*record).flag ^= __FLAG__OCCLUDED__;
                            (*record).occlusion_frame_count = 0;
                            (*point).back().tracking_error = 0.0;
                            cout << endl<< " - point " << i << " unoccluded" ;
                        }
                    } catch (...) {
#ifdef __COUT__LEVEL__1
                        cout << " - point " << i << " too close to border" << endl;
#endif
                    }
                }
            }
            else if ((*record).flag == __FLAG__INITIALIZED__) { //point is in inlier set
#ifndef __ESTIMATE__STRUCTURE__ONCE__
                if (inlier_ratio*100>__THRESHOLD__INLIER__SET__) { //if 50% are being nicely tracked then modify them
                    tracklets_for_structure_estimation.push_back(i);
                }
#endif
                //find projected locations
                //                Point2f x_prev =tracklet_record.at(i).point.at(tracklet_record[i].life-2).position;
                //                Mat uv = image_buffer.at(curr_image_index).camera*tracklet_record.at(i).structure;
                //                tracklet_record.at(i).point.at(tracklet_record[i].life-1).position.x = uv.at<double>(0)+x_prev.x;
                //                tracklet_record.at(i).point.at(tracklet_record[i].life-1).position.y = uv.at<double>(1)+x_prev.y;
            }
            else if ((*record).flag == __FLAG__OCCLUDED__) {
                //got occluded before structure is initialized so erase the point
#ifdef __COUT__LEVEL__2__
                cout << " - Erased "<<i<<endl;
#endif
                tracklet_record.erase(tracklet_record.begin()+i);
                i--; //since erased we need to subtract 1 from i
            }
            else {
                //if new point with adequate history push to control array for structure information
                if ((*record).life>__FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__){
                    tracklets_for_structure_estimation.push_back(i);
                }
            }
        }
        final_tracklet_count = tracklet_record.size();
        //-----------------------------------------------------------------------------------------------
        //if longtime occluded points are erased then add new points
//        int number_of_removed_points = (int)(initial_tracklet_count-final_tracklet_count);
//        if (number_of_removed_points>0) {
//            cout<<"Replacing "<<initial_tracklet_count-final_tracklet_count<<" points\n";
        Add_New_Points();
//        }
        Estimate_Structure(tracklets_for_structure_estimation);
        //Reproject points using structure
//        for (int i = 0 ; i < tracklet_record.size();i++){
//            if ( (tracklet_record.at(i).flag & __FLAG__INITIALIZED__)>0) { //point is in inlier set
//                Point2f x_prev =tracklet_record.at(i).point.at(tracklet_record.at(i).life-2).position;
//                //draw projected location
//                Mat uv = image_buffer.at(curr_image_index).camera*tracklet_record.at(i).structure;
//                tracklet_record.at(i).point.back().position.x=uv.at<double>(0)+x_prev.x;
//                tracklet_record.at(i).point.back().position.y=uv.at<double>(1)+x_prev.y;
//            }
//        }
    }
    else if (processed_image_count == __IMAGE__BUFFER__SIZE__) {
        Subspace_Geometry_Initialize_From_Motion();
    }
}


//void cTrackingFramework::Add_New_Points(long count){
void cTrackingFramework::Add_New_Points(){
    MY_POINT my_point;
    int curr_image_index = (__IMAGE__BUFFER__SIZE__+processed_image_count-1) % __IMAGE__BUFFER__SIZE__;
    int rows = image_buffer.at(curr_image_index).gray_image.rows;
    int cols = image_buffer.at(curr_image_index).gray_image.cols;
    bool *control=new bool[rows*cols];
    //set borders to 1
    memset((void*)control, true, rows*cols*sizeof(bool));
    for (int i=__BORDER__LIMIT__;i<rows-__BORDER__LIMIT__;i++) {
        memset((void*)(control+i*cols+__BORDER__LIMIT__), false, (cols-__BORDER__LIMIT__)*sizeof(bool));
    }
    Point2f point;
    for (int i=0;i<tracklet_record.size();i++){
        point = tracklet_record.at(i).point.back().position;
        float scale = 1.0 + (tracklet_record.at(i).flag == __FLAG__OCCLUDED__INITIALIZED__);
        int x_start = max(0.0, point.x-scale*10.0);
        int x_end   = min(cols-1.0, point.x+scale*10.0);
        int x_count = max(0,x_end-x_start);
        int y_start = max(0.0, point.y-scale*10.0);
        int y_end   = min(rows-1.0, point.y+scale*10.0);
        for (int y=y_start;y<=y_end;y++) {
            memset((void*)&control[y*cols+x_start], true, x_count*sizeof(bool));
        }
    }
//    Mat tmpim(rows,cols,CV_8U,255);
//    for (int i=0;i<rows;i++) for (int j=0;j<cols;j++) if (control[i*cols+j]) tmpim.at<char>(i,j)=255; else tmpim.at<char>(i,j)=0;
//    imwrite("/Users/yilmaz.15/Desktop/im.jpg", tmpim);

//    int corner_index=0;
    TRACKLET_RECORD new_trajectory;
    new_trajectory.flag = __FLAG__NONE__;
    new_trajectory.occlusion_frame_count = 0;
    new_trajectory.starting_frame_no=processed_image_count-1;
    new_trajectory.life=1;
    my_point.tracking_error = 0;
    for (int i=0;i<image_buffer.at(curr_image_index).corners.size();i++) {
        int x,y;
        point = image_buffer.at(curr_image_index).corners.at(i);
        x=point.x;
        y=point.y;
        bool occupied = control[y*cols+x];

        if (!occupied) {
            new_trajectory.color_code = Scalar(rng.uniform(0,256),rng.uniform(0,256),rng.uniform(0,256));
            new_trajectory.point.clear();
            my_point.position = point;
            new_trajectory.point.push_back(my_point);
            tracklet_record.push_back(new_trajectory);
            //mark out the selected region in the control set to abunden selection of points in the vicinity
            int x_start = max(0.0, point.x-10.0);
            int x_end   = min(cols-1.0, point.x+10.0);
            int x_count = max(0,x_end-x_start);
            int y_start = max(0.0, point.y-10.0);
            int y_end   = min(rows-1.0, point.y+10.0);
            for (int y=y_start;y<=y_end;y++) {
                memset((void*)&control[y*cols+x_start], true, x_count*sizeof(bool));
            }
#ifdef __COUT__LEVEL__2__
            cout << " - Adding ("<<my_point.position.x<<","<<my_point.position.y<<")"<<endl;
#endif
        }
//        else {
//            cout<< " - Cannot add any points";
//        }
    }
    delete [] control;
}

void cTrackingFramework::Subspace_Geometry_Initialize_From_Motion(){
    //use only the points that are unoccluded in this frame that have 3D locations
    Mat camera_matrices;
    Mat locations;
    vector< vector<double> > motion_observations;
    SVD svd;
    string str;
    //remove the points that are already occluded
    for (int i=0;i<tracklet_record.size();i++){
        if ( (tracklet_record.at(i).flag & __FLAG__OCCLUDED__) >0 ) {
#ifdef __COUT__LEVEL__2__
            cout << " - Erased "<<i<<endl;
#endif
            tracklet_record.erase(tracklet_record.begin()+i);
            i--; //since the current one is erased you need to reduce the inex by one
        }
    }
    //compute the observation matrix
    for (int i=0;i<tracklet_record.size();i++){
        if (tracklet_record.at(i).point.size()==processed_image_count) {
            vector<double> u;
            vector<double> v;
            Point2f u_vector;
            for (int j=1;j<tracklet_record.at(i).point.size();j++) { //start from one to compute optical flow
                u_vector = tracklet_record.at(i).point.at(j  ).position-tracklet_record.at(i).point.at(j-1).position;
                u.push_back(u_vector.x);
                v.push_back(u_vector.y);
            }
            motion_observations.push_back(u);
            motion_observations.push_back(v);
        }
        else {
            cout <<"   - skipping "<<i<<endl;
        }
    }
    Mat observation_matrix(2*(int)motion_observations.at(0).size(),(int)motion_observations.size()/2, DataType<double>::type);
    //organize the observation data
    
    for (int i=0;i<motion_observations.size()/2;i++) {
        for (int j=0;j<(int)motion_observations.at(0).size();j++) {
            observation_matrix.at<double>(2*j  ,i) = motion_observations.at(2*i  ).at(j);
            observation_matrix.at<double>(2*j+1,i) = motion_observations.at(2*i+1).at(j);
        }
    }
//
//    Mat tmp_mat((int)motion_observations.size(), (int)motion_observations.at(0).size(), DataType<double>::type);
//    for (int i=0;i<tmp_mat.rows;i++) {
//        for (int j=0;j<tmp_mat.cols;j++) {
//            tmp_mat.at<double>(i,j) = motion_observations.at(i).at(j);
//        }
//    }

    
//    cout<<"mo="<<tmp_mat<<";"<<endl<<endl;
//    cout<<"om="<<observation_matrix<<";"<<endl<<endl;
    //compute SVD
    try {
//        cout << "obs="<<observation_matrix;
        svd(observation_matrix);//,SVD::FULL_UV);
        // use 6 of the dimensions from the V matrix to store the camera matrix
        for (int i=1;i<image_buffer.size();i++) {
            for (int j=0;j<__SUBSPACE__DIMENSION__;j++){
                image_buffer.at(i).camera.at<double>(0,j)=svd.u.at<double>(2*(i-1)  ,j);
                image_buffer.at(i).camera.at<double>(1,j)=svd.u.at<double>(2*(i-1)+1,j);
            }
        }
        Mat S_Vt = Mat::diag(svd.w)*svd.vt;
        Mat Vt(S_Vt,Range(0,DCT_space.rows),Range(0,svd.vt.cols));
//        cout <<"dct="<< DCT_space_right_side<<endl<<endl;
//        cout <<"w="<< Mat::diag(svd.w)<<endl<<endl;
//        cout <<"vt="<< svd.vt<<endl<<endl;
//        cout <<"S_Vt="<< S_Vt<<endl<<endl;
//        cout <<"Vt="<< Vt<<endl<<endl;
        
        // use 6 of the dimensions from the U matrix to store the structure vector
        for (int i=0;i<tracklet_record.size();i++) {
            tracklet_record.at(i).DCT_coefficients = Mat(DCT_space.cols,1,CV_64F,0.0);
            if (tracklet_record.at(i).point.size()==processed_image_count) {
                tracklet_record.at(i).flag |= __FLAG__INITIALIZED__;
                
                tracklet_record.at(i).DCT_coefficients = DCT_space_right_side*Vt.col(i);
            }
        }
    } catch (...) {
        cout << " - Cannot initialize the structure"<<endl;
        exit(-1);
    }
//    cout << "CAMR = " << " "  << image_buffer.at(0).camera << endl << endl;
//    cout << "STRM = " << " "  << tracklet_record.at(0).structure << endl << endl;
//    cout << "u = "<<motion_observations.at(0).at(0)<<endl;
//    cout << "v = "<<motion_observations.at(1).at(0)<<endl;
    
}

//#ifdef __ESTIMATE__STRUCTURE__ONCE__
void cTrackingFramework::Estimate_Structure(vector <int> &tracklets_for_structure_estimation) {
    if (tracklets_for_structure_estimation.size()>0) {
        Point2f u_vector;
        int N = __FRAMES__TO__COMPUTE_STRUCTURE__NEW__POINTS__;//(int)tracklet_record.at(point_index).point.size();
        Mat Sol;
        Mat B(2*(N),1,CV_64F);
        Mat A(2*(N),DCT_space.cols,CV_64F);
        
        //  estimate structure of the new points
        for (int i = 0 ; i < tracklets_for_structure_estimation.size();i++){
            int point_index = tracklets_for_structure_estimation.at(i);
            TRACKLET_RECORD *record = &tracklet_record.at(point_index);
            vector <MY_POINT> *points = &((*record).point);
            
            int tracklet_start_index = (int)(*points).size()-N;
            int image_start_index = processed_image_count-N;//(int)tracklet_record.at(point_index).starting_frame_no+tracklet_start_index;
            for (int j=0;j<N;j++) { //start from one to compute optical flow
                double weight_track_error  =
                    exp(-(*points).at(tracklet_start_index+j).tracking_error * (*points).at(tracklet_start_index+j).tracking_error/0.9);
                u_vector = (*points).at(tracklet_start_index+j).position - (*points).at(tracklet_start_index+j-1).position;
                B.at<double>(2*(j)  ) = weight_track_error*u_vector.x;
                B.at<double>(2*(j)+1) = weight_track_error*u_vector.y;
                
                int im_no = (__IMAGE__BUFFER__SIZE__+j+image_start_index) % __IMAGE__BUFFER__SIZE__;
                Mat camera_dct = image_buffer.at(im_no).camera * DCT_space;
                
                for (int k=0;k<DCT_space.cols; k++) {
                    A.at<double>(2*(j)  ,k) = weight_track_error*camera_dct.at<double>(0,k);
                    A.at<double>(2*(j)+1,k) = weight_track_error*camera_dct.at<double>(1,k);
                }
            }
            try {
                Mat Sol;
                solve(A, B, Sol, DECOMP_QR);
                Mat error_reprojection = B-A*Sol;
                double error_magnitute = norm(error_reprojection);
                if (error_magnitute < __THRESHOLD__REPROJECTION__ERROR__) {
                    (*record).flag |= __FLAG__INITIALIZED__;
                    (*record).DCT_coefficients = Sol;
                }
                else {
                    (*record).flag |= __FLAG__OCCLUDED__;
                    cout<<endl<<" - error: "<<error_magnitute<<"  cannot set structure for tracklet "<<point_index;
                }
            } catch (...) {
                cout << endl<< " - Cannot estimate structure for tracklet "<<point_index<<endl;
            }
        }
    }
}

Mat cTrackingFramework::Estimate_Camera(vector<int> &indices) {
    double u,v;
    Mat B(2*(int)indices.size(),1,CV_64F);
    Mat A(2*(int)indices.size(),2*__SUBSPACE__DIMENSION__,CV_64F,0.0);
    
    //for computing weights
    double total_life=0;
    for (int i=0;i<indices.size();i++) {
        int index = indices.at(i);
        total_life += tracklet_record.at(index).life;
    }
    
    for (int i=0;i<indices.size();i++) {
        int index = indices.at(i);
        TRACKLET_RECORD *record = &tracklet_record.at(index);
        vector <MY_POINT> *points = &((*record).point);

        int j = (int)(*points).size()-1;
        double weight_life  = 1-__WEIGHTED__LEAST__SQUARES__+ __WEIGHTED__LEAST__SQUARES__*(*record).life/total_life;
        double weight_track_error  =
                    exp(-(*points).at(j).tracking_error*(*points).at(j).tracking_error/0.1);
        double weight = 1 - __WEIGHTED__LEAST__SQUARES__ + __WEIGHTED__LEAST__SQUARES__*weight_life*weight_track_error;
        u = (*points).at(j).position.x - (*points).at(j-1).position.x;
        v = (*points).at(j).position.y - (*points).at(j-1).position.y;

        B.at<double>(2*i  )=weight*u;
        B.at<double>(2*i+1)=weight*v;
        
        Mat dct_times_coef = DCT_space*(*record).DCT_coefficients;
        
        for (int j=0;j<__SUBSPACE__DIMENSION__;j++) {
            double str_val = dct_times_coef.at<double>(j);
            A.at<double>(2*i  ,j)= weight * str_val;
            A.at<double>(2*i+1,j+__SUBSPACE__DIMENSION__) = weight * str_val;
        }
    }
    Mat Sol;
    try {
        solve(A, B, Sol, DECOMP_QR);
    } catch (...) {
        cout<<" - Cannot estimate camera parameters";
        exit(-1);
    }
    
//    if (processed_image_count==32) {
//        for (int i=0;i<tracklet_record.size();i++) {
//            cout << "str"<<i<<" = "<<tracklet_record.at(i).structure<<endl<<endl;
//        }
//    }
//    if (processed_image_count>30) {
//        cout  << "A = "<<  " "  << A << endl << endl;
//        cout  << "B = "<<  " "  << B << endl << endl;
//        cout  << "S = "<<  " "  << Sol << endl << endl;
//    }
    return Sol.reshape(0, 2);
}

double cTrackingFramework::Test_Camera(Mat &camera_DCT, Mat &DCT_coefficients,Point2f &prev,Point2f &curr){
    Mat uv_estimate = camera_DCT*DCT_coefficients;
    Point2f uv_computed = curr - prev;
    float x = uv_computed.x-uv_estimate.at<double>(0);
    float y = uv_computed.y-uv_estimate.at<double>(1);
    return  x*x + y*y ;
}


vector<int> cTrackingFramework::Generate_Inliers_For_New_Frame_RANSAC() {
    
    vector <Mat> structure;
    vector <Point2f> uv;
    
    vector <int> indices_of_active_points;
    
    for (int i=0; i<tracklet_record.size(); i++) {
        if (tracklet_record.at(i).flag == __FLAG__INITIALIZED__) {
            indices_of_active_points.push_back(i);
        }
    }
//    cout << "Frame no " << setprecision(3) << processed_image_count-1 << ". Inlier set count " << setprecision(4) <<indices_of_active_points.size() << endl;

    double N=1;      //Dummy initialisation for number of trials.
    double trialcount = 0;
    double p = 0.99; //Desired probability of choosing at least one sample
    int max_sample_size = (int)indices_of_active_points.size();
    Mat estimated_camera_DCT_space;
    vector<int> maximum_inlier_set_indices;
    
    while (N>trialcount ) {
        //Select at random s datapoints to form a trial model, M.
        //Generate s random indicies in the range 1..npts
        //Find corresponding tracklet indices from random indices
        vector <int> tracklet_indices(__RANSAC__SAMPLE__SIZE__,1);
        for (int i=0;i<__RANSAC__SAMPLE__SIZE__;i++){
            int index = rng.uniform(0,max_sample_size);
            tracklet_indices.at(i)=indices_of_active_points.at(index);
        }
        //Fit model to this random selection of data points.
        estimated_camera_DCT_space = Estimate_Camera(tracklet_indices)*DCT_space;
        
        //Select inlier set by evaluating distances
        vector<int> inlier_set_indices;// = tracklet_indices;
        for (int i=0;i<indices_of_active_points.size();i++) {
            int index = indices_of_active_points.at(i);
            vector <MY_POINT> *points = &tracklet_record.at(index).point;
            
            int j = (int)(*points).size()-1;
            Point2f curr_point = (*points).at(j  ).position;
            Point2f prev_point = (*points).at(j-1).position;
            double dist = Test_Camera(estimated_camera_DCT_space, tracklet_record.at(index).DCT_coefficients, prev_point, curr_point);
            if (dist<__THRESHOLD__RANSAC__DISTANCE__) {
                inlier_set_indices.push_back(index);
            }
        }
        if (inlier_set_indices.size()>maximum_inlier_set_indices.size()) {
            maximum_inlier_set_indices = inlier_set_indices;
            double fracinliers =  (double)maximum_inlier_set_indices.size()/(double)indices_of_active_points.size();
            double pNoOutliers = 1 -  pow(fracinliers,ceil(__SUBSPACE__DIMENSION__/2));
            pNoOutliers = max(1e-10, pNoOutliers);  // Avoid division by -Inf
            pNoOutliers = min(1-1e-10, pNoOutliers);// Avoid division by 0.
            //Update estimate of N, the number of trials to ensure we pick, with probability p, a data set with no outliers.
            N = log(1-p)/log(pNoOutliers);
        }
        trialcount = trialcount+1;
        //Safeguard against being stuck in this loop forever
        if (trialcount > __RANSAC__MAX__TRIAL__COUNT__) {
            cout << "ransac reached the maximum number of %d trials"<< endl;
            N=trialcount-1;
        }
    }
    return maximum_inlier_set_indices;
}

























