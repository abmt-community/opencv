#ifndef OPENCV_MARKER_H
#define OPENCV_MARKER_H OPENCV_MARKER_H

#include <opencv2/objdetect/aruco_detector.hpp>

#include "calib.h"

//@link: opencv_objdetect;

namespace opencv{

//@node: auto
//@raster: auto
struct detect_markers{
    calib_data param_cam = {"../cam_undist.json"};
    abmt::img in_img;
    std::vector<int> out_ids;
    // may contain empty corners list for easy use with solve_pnp
    std::vector<std::vector<abmt::vec2>> out_corners;
    
    
    cv::aruco::PredefinedDictionaryType param_type = cv::aruco::DICT_4X4_50;
    
    cv::aruco::ArucoDetector detector;
    void init();
    void tick();
};

//@node: auto
//@raster: auto
struct marker_points{
    std::vector<abmt::vec3> out;
    double param_border_len = 0.06;
    void init();
};

} // namespace opencv

#endif // OPENCV_MARKER_H
