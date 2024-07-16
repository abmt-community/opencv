#ifndef BUS_OPENCV_H
#define BUS_OPENCV_H BUS_OPENCV_H

#include <abmt/img.h>
#include <opencv2/core/mat.hpp>
#include <abmt/math/mat.h>
#include <abmt/math/vec.h>
#include <vector>

namespace opencv{

//@link: opencv_core
//@link: opencv_imgproc

abmt::img mat2img(cv::Mat& m);
cv::Mat img2mat(abmt::img& img);

cv::Mat a2c(abmt::vec2& in);
cv::Mat a2c(abmt::vec3& in);
cv::Mat a2c(std::vector<abmt::vec2>& in);
cv::Mat a2c(std::vector<abmt::vec3>& in);
cv::Mat a2c(abmt::img& img);

void c2a(cv::Mat& m, abmt::vec2& v);
void c2a(cv::Mat& m, abmt::vec3& v);
void c2a(cv::Mat& m, abmt::mat3& ma);
void c2a(cv::Mat& m, abmt::img& img);


//@node: auto
struct blur {
    abmt::img_gray in_img;
    int            in_blur_size = 10;
    abmt::img_gray out;
    
    void tick();
};


//@node: auto
struct blur_rgb {
    abmt::img_rgb in_img;
    int           in_blur_size = 10;
    abmt::img_rgb out;
    
    void tick();
};


//@node: auto
struct canny {
    abmt::img_gray in_img;
    double         in_threshold1 = 50;
    double         in_threshold2 = 50;
    abmt::img_gray out;

    void tick();
};


//@node: auto
struct hough_lines {
    abmt::img_gray            in;
    std::vector<abmt::line2d> out;
    
    void tick();
};


} // namespace bus

#endif // BUS_OPENCV_H