#ifndef OPENCV_MARKER_H
#define OPENCV_MARKER_H OPENCV_MARKER_H

#include "calib.h"

namespace opencv{

//@node: auto
//@raster: auto
struct detect_markers{
    calib_data param_cam = {"../cam_undist.json"};
    abmt::img in_img;
    void init();
    void tick();
};

} // namespace opencv

#endif // OPENCV_MARKER_H
