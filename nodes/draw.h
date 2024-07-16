#ifndef OPENCV_DRAW_H
#define OPENCV_DRAW_H OPENCV_DRAW_H

#include <vector>
#include <abmt/math/vec.h>
#include "opencv.h"

namespace opencv{

//@node: auto
//@raster: auto
struct draw_points{
    std::vector<abmt::vec2> in_point_list;
    abmt::img_rgb in_img;
    int param_line_width = 1;
    abmt::vec3 param_color = {0,255,0};
    void tick();
};

} // namespace opencv

#endif // OPENCV_DRAW_H
