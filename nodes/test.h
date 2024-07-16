#ifndef OPENCV_TEST_H
#define OPENCV_TEST_H OPENCV_TEST_H

#include "opencv.h"
#include "calib.h"

//@link: opencv_core
//@link: opencv_imgproc
//@link: opencv_calib3d
//@compile: opencv.cpp
//@compile: calib.cpp

namespace opencv{

//@node: auto
//@raster: auto
struct img2plane{
    abmt::vec3 out = {0,0,0};
    abmt::vec3 out_world = {0,0,0};
    abmt::vec2 in;
    abmt::pose in_w2c;
    abmt::pose in_c2w;
    
    calib_data param_cam = {"../cam_undist.json"};
    abmt::vec3 param_plane_normal = {0,0,1};
    abmt::vec3 param_plane_point  = {0,0,0};
    
    abmt::vec3 n0;
    abmt::vec4 plane_point4;
    abmt::vec4 plane_normal4;
    double d;
    
    void init();
    void tick();
};


//@node: auto
//@raster: auto
struct world_2_img{
    abmt::vec3 in = {0,0,0};
    abmt::vec2 out;
    abmt::pose in_world;
    
    calib_data param_cam = {"../cam_undist.json"};
    abmt::mat<3,4,double> w2c = {
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,0}
    };
    void tick();
};


//@node: auto
//@raster: auto
struct inverse_pose{
    abmt::pose in;
    abmt::pose out;
    void tick();
};

} // namespace opencv

#endif // OPENCV_TEST_H
