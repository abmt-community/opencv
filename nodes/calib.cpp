#include "calib.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <abmt/json.h>
#include <filesystem>
#include <fstream>

using namespace std;
using namespace opencv;
using namespace cv;
using namespace abmt;

calib_data::calib_data(std::string json_file_name){
	abmt::die_if(std::filesystem::exists(json_file_name) == false, "Unable to open file: " + json_file_name);
	ifstream f(json_file_name.c_str());
	stringstream ss;
	ss << f.rdbuf();
	f.close();
	from_json( json::parse( ss.str() ) );
}

void calib_data::write_to(std::string json_file_name){
    auto str = to_json().dump_compact();
    auto file = fopen(json_file_name.c_str(), "wb");
    abmt::die_if(file == 0, "record: unable to open " + json_file_name);
    fwrite(str.c_str(),str.size(),1,(FILE*)file);
    fclose((FILE*)file);
}

abmt::json calib_data::to_json(){
    json obj;
    obj["mat"] = json::array();
    for(int i = 0; i < 3; i++){
        auto row = json::array();
        for(int j = 0; j < 3; j++){
            row.push_back(mat.at<double>(i,j));
        }
        obj["mat"].push_back(row);
    }
    obj["dist"] = json::array();
    for(auto k: dist){
        obj["dist"].push_back(k);
    }
    obj["rms"] = rms;
    return obj;
}

void calib_data::from_json(abmt::json obj){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            mat.at<double>(i,j) = obj["mat"][i][j];
            mat3[i][j] = (double)obj["mat"][i][j];
        }
    }
    for(auto k: obj["dist"]){
        dist.push_back(k.value);
    }
    rms = obj["rms"];
}


void calib::tick(){
    if(in_add_corners && in_points_img.size() > 0){
        std::vector<cv::Point2f> points;
        for(auto&p: in_points_img){
            points.push_back({(float)p.x, (float)p.y});
        }
        img_points.push_back(points);
        
        std::vector<cv::Point3f> points_o;
        for(auto&p: in_points_obj){
            points_o.push_back({(float)p.x, (float)p.y, (float)p.z});
        }
        obj_points.push_back(points_o);
    }
}

void calib::final(){
    abmt::log("start calc: num img: " + to_string(img_points.size()));
    calib_data cam;
    Mat rvecs;
    Mat tvecs;
    Mat new_obj_points;
    int new_method = -1;
    if(param_new_method){
        new_method = 1;
    }
    
    double res = calibrateCameraRO(obj_points, img_points, Size(in_img.width, in_img.height), new_method, cam.mat, cam.dist, rvecs, tvecs, new_obj_points);
    cam.rms = res;
    abmt::log("mat:     " + cam.to_json()["mat"].dump_compact());
    abmt::log("dist:    " + cam.to_json()["dist"].dump_compact());
    abmt::log("rms_err: " + to_string(res));
    cam.write_to(param_cam_json);
    
    calib_data cam_undist;
    cam_undist.mat = cv::getOptimalNewCameraMatrix(cam.mat, cam.dist, Size(in_img.width, in_img.height), param_undist_alpha);
    cam_undist.write_to(param_cam_json_undistort);
    
}


void find_chessboard::init(){
    // chessboard row by row. From left to right
    for (int j = 0; j < param_chess_n_elems_y; ++j) {
        for (int i = 0; i < param_chess_n_elems_x; ++i) {
            out_corners_obj.push_back({i*param_chess_elem_len, j*param_chess_elem_len, 0});
            //out_corners_obj.push_back({0, j*param_chess_elem_len, i*param_chess_elem_len,});
        }
    }
}

void find_chessboard::tick(){
    vector<Point2f> res;
    int flags = CALIB_CB_ADAPTIVE_THRESH & CALIB_CB_NORMALIZE_IMAGE & CALIB_CB_FAST_CHECK;
    out_found = findChessboardCorners(img2mat(in_img), {param_chess_n_elems_x, param_chess_n_elems_y}, res, flags);
 
    out_corners_img.clear();
    if(out_found){
        if(praem_subpix){
            cornerSubPix( img2mat(in_img), res, Size(param_sub_pix_size,param_sub_pix_size), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
        }
        for(auto& p: res){
            out_corners_img.push_back({p.x, p.y});
        }
    }
}


void undistort::init(){
    cam = calib_data(param_cam_json);
    abmt::log("cam: " + cam.to_json().dump_compact());
    cam_undist = calib_data(param_cam_json_new);
}

void undistort::tick(){
    Mat out;
    cv::undistort(img2mat(in_in_img), out, cam.mat, cam.dist, cam_undist.mat);
    out_out_img = mat2img(out);
}


void solve_pnp::tick(){
    out_rvec = {};
    out_tvec = {};
    out_pose = pose_zero;
    if(in_points_img.size() >= 4){
        Mat rvec;
        Mat tvec;
        //cv::solvePnP(a2c(in_points_obj), a2c(in_points_img), param_cam.mat, param_cam.dist, rvec, tvec, false, cv::SOLVEPNP_IPPE );
        cv::solvePnP(a2c(in_points_obj), a2c(in_points_img), param_cam.mat, param_cam.dist, rvec, tvec );
        c2a(rvec, out_rvec);
        c2a(tvec, out_tvec);
        abmt::log("found: " + to_string(out_tvec[0]) + " " + to_string(out_tvec[1]) + " " + to_string(out_tvec[2]));
        
        mat3 ma;
        Mat mc = cv::Mat::eye(3, 3, CV_64F);
        cv::Rodrigues(rvec, mc);
        c2a(mc, ma);
        out_pose = ma;
        out_pose.x = out_tvec.x;
        out_pose.y = out_tvec.y;
        out_pose.z = out_tvec.z;
    }
}


void draw_frame_axis::tick(){
    out_out_img = in_in_img.copy();
    cv::drawFrameAxes(a2c(out_out_img), param_cam.mat, param_cam.dist, a2c(in_rvec), a2c(in_tvec), param_len, param_thickness);
}


void cam_matrix::init(){
    out_mat3 = param_cam.mat3;
}

