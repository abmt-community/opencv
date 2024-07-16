#include "test.h"

using namespace std;
using namespace opencv;
using namespace abmt;

void img2plane::init(){
    plane_point4 = {param_plane_point.x, param_plane_point.y, param_plane_point.z, 1};
}

void img2plane::tick(){
    auto norm_pn = param_plane_normal.norm();
    plane_normal4 = {norm_pn.x, norm_pn.y, norm_pn.z, 0}; // 0 -> rotate only;
    auto n0_tmp = in_w2c*plane_normal4;
    n0 = {n0_tmp.x, n0_tmp.y, n0_tmp.z};
    
    auto p4_trans = in_w2c*plane_point4;
    vec3 p3_trans = {p4_trans.x, p4_trans.y, p4_trans.z};
    d = (n0 * p3_trans).sum();
    double f_x = param_cam.mat3[0][0];
    double f_y = param_cam.mat3[1][1];
    double c_x = param_cam.mat3[0][2];
    double c_y = param_cam.mat3[1][2];
    double e_x = n0.x;
    double e_y = n0.y;
    double e_z = n0.z;
    double u = in.x;
    double v = in.y;
    // line equation inserted in plane equation
    double z = d*f_x*f_y/(-c_x*e_x*f_y - c_y*e_y*f_x + e_x*f_y*u + e_y*f_x*v + e_z*f_x*f_y);
    out.x = (u-c_x)/f_x * z;
    out.y = (v-c_y)/f_y * z;
    out.z = z;
    abmt::log("cam   x: " + to_string(out.x) + "  y: " + to_string(out.y) +"  z: " + to_string(out.z));
    vec4 cam_ray_pos = {out.x, out.y, out.z, 1};
    vec4 world_sp = in_c2w*cam_ray_pos;
    
    abmt::log("world x: " + to_string(world_sp.x) + "  y: " + to_string(world_sp.y) +"  z: " + to_string(world_sp.z));
    
    
}



void world_2_img::tick(){
    vec4 in_w = { in.x, in.y, in.z, 1 };
    auto to_cam = w2c*in_world*in_w;
    if(to_cam.z > 0){
        out.x = param_cam.mat3[0][0]*to_cam.x/to_cam.z + param_cam.mat3[0][2];
        out.y = param_cam.mat3[1][1]*to_cam.y/to_cam.z + param_cam.mat3[1][2];
    }
    auto t = out;
    abmt::log("w2i: " + to_string( t.x) + " " + to_string(t.y) );
}


void inverse_pose::tick(){
    abmt::mat3 rot;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            rot[i][j] = in[i][j];
        }
    }
    vec3 t = {in.x, in.y, in.z};
    t = rot.t()*t;
    //out = -in;
    out = rot.t();
    out.x = -t.x;
    out.y = -t.y;
    out.z = -t.z;
    abmt::log("new_t: " + to_string(out.x) + " " + to_string(out.y) + " " + to_string(out.z));
        
}

