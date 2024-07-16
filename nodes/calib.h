#ifndef OPENCV_CALIB_H
#define OPENCV_CALIB_H OPENCV_CALIB_H

#include <vector>
#include <string>
#include <abmt/math/vec.h>
#include <abmt/math/mat.h>
#include <abmt/json.h>
#include <opencv2/core/mat.hpp>
#include "opencv.h"

//@link: opencv_core
//@link: opencv_imgproc
//@link: opencv_calib3d
//@compile: opencv.cpp

namespace opencv{


struct calib_data{
    cv::Mat    mat  = cv::Mat::eye(3, 3, CV_64F);
    abmt::mat3 mat3; // only loaded in from_json
    std::vector<double> dist;
    double rms = 0;
    
    calib_data(){};
    calib_data(std::string json_file_name);
    void write_to(std::string json_file_name);
    abmt::json to_json();
    void from_json(abmt::json);
};


//@node: auto
//@raster: auto
struct calib{
    std::vector<abmt::vec2> in_points_img;
    std::vector<abmt::vec3> in_points_obj;
    bool in_add_corners = true;
    abmt::img_gray in_img;
    
    bool param_new_method=true;
    std::string param_cam_json = "../cam.json";
    std::string param_cam_json_undistort = "../cam_undist.json";
    double param_undist_alpha = 0;
    
    std::vector<std::vector<cv::Point2f>> img_points;
    std::vector<std::vector<cv::Point3f>> obj_points;
    
    void tick();
    void final();
};


//@node: auto
//@raster: auto
struct find_chessboard{
    
    abmt::img_gray in_img;
    std::vector<abmt::vec2> out_corners_img;
    std::vector<abmt::vec3> out_corners_obj;
    bool out_found;
    
    int    param_chess_n_elems_x = 8;
    int    param_chess_n_elems_y = 6;
    double param_chess_elem_len  = 0.025;
    bool   praem_subpix          = true;
    int    param_sub_pix_size    = 11;
    
    void init();
    void tick();
};


//@node: auto
//@raster: auto
struct undistort{
    abmt::img in_in_img;
    abmt::img out_out_img;
    std::string param_cam_json = "../cam.json";
    std::string param_cam_json_new = "../cam_undist.json";
    calib_data cam;
    calib_data cam_undist;
    
    void init();
    void tick();
};


//@node: auto
//@raster: auto
struct solve_pnp{
    calib_data param_cam = {"../cam_undist.json"};
    
    std::vector<abmt::vec2> in_points_img;
    std::vector<abmt::vec3> in_points_obj;
    
    abmt::vec3 out_rvec;
    abmt::vec3 out_tvec;
    abmt::pose out_pose;
    abmt::pose pose_zero;
    
    void tick();
};


//@node: auto
//@raster: auto
struct draw_frame_axis{
    abmt::vec3 in_rvec;
    abmt::vec3 in_tvec;
    abmt::img in_in_img;
    abmt::img out_out_img;
    
    calib_data param_cam = {"../cam_undist.json"};
    float      param_len = 1;
    int        param_thickness = 3;
    
    void tick();
};


//@node: auto
//@raster: auto
struct cam_matrix{
    calib_data param_cam = {"../cam_undist.json"};
    abmt::mat3 out_mat3;
    void init();
};


} // namespace opencv

#endif // OPENCV_CALIB_H
