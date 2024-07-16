#include "draw.h"

using namespace std;
using namespace opencv;

void draw_points::tick(){
    for(auto& p:in_point_list){
        in_img.draw_cross(p, param_color, param_line_width);
    }
}

