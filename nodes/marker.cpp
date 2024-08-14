#include "marker.h"

#include "opencv.h"


using namespace std;
using namespace opencv;

void detect_markers::init(){
   
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(param_type);
    detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
    
}

void detect_markers::tick(){
    out_ids.clear();
    out_corners.clear();
    std::vector<std::vector<cv::Point2f>> cv_corners;
    detector.detectMarkers(img2mat(in_img), cv_corners, out_ids);
    for(auto v: cv_corners){
        std::vector<abmt::vec2> corners;
        for(auto c: v){
            corners.push_back({c.x, c.y});
        }
        out_corners.push_back(corners);
    }
    abmt::log("found markers: " + std::to_string(out_ids.size()));
    if( out_ids.size() > 0 ){
        //abmt::log("id: " + std::to_string(out_ids[0]));
    }else{
        // add empty corners for easy use with solve_pnp
        std::vector<abmt::vec2> corners;
        out_corners.push_back(corners);
    }
}


void marker_points::init(){
    out.push_back({0,0,0});
    out.push_back({param_border_len,0,0});
    out.push_back({0,param_border_len,0});
    out.push_back({param_border_len,param_border_len,0});
}
