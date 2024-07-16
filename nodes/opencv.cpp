#include "opencv.h"

#include <abmt/blob.h>
#include <iostream>
#include <abmt/math/vec.h>
#include <abmt/math/ray.h>
#include <opencv2/imgproc.hpp>
#include <abmt/time.h>

using namespace std;
using namespace opencv;
using namespace abmt;
using namespace cv;

cv::Mat opencv::a2c(abmt::vec2& in){
    cv::Point2f res = {(float)in.x, (float)in.y};
    return Mat(res,true);
}

cv::Mat opencv::a2c(abmt::vec3& in){
    cv::Point3f res = {(float)in.x, (float)in.y, (float)in.z};
    return Mat(res,true);
}

cv::Mat opencv::a2c(std::vector<abmt::vec2>& in){
    std::vector<cv::Point2f> res;
    for(auto&p: in){
        res.push_back({(float)p.x, (float)p.y});
    }
    return Mat(res,true);
}

cv::Mat opencv::a2c(std::vector<abmt::vec3>& in){
    std::vector<cv::Point3f> res;
    for(auto&p: in){
        res.push_back({(float)p.x, (float)p.y, (float)p.z});
    }
    return Mat(res,true);
}

cv::Mat opencv::a2c(abmt::img& img){
    return img2mat(img);
}

void opencv::c2a(cv::Mat& m, abmt::vec2& v){
    for(int i = 0; i < 2; i++){
        v[i] = m.at<double>(i,0);
    }
}

void opencv::c2a(cv::Mat& m, abmt::vec3& v){
    for(int i = 0; i < 3; i++){
        v[i] = m.at<double>(i,0);
    }
}

void opencv::c2a(cv::Mat& mc, abmt::mat3& ma){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            ma[i][j] = mc.at<double>(i,j);
        }
    }
}

void opencv::c2a(cv::Mat& m, abmt::img& img){
    img = mat2img(m);
}

abmt::img opencv::mat2img(Mat& m){
    abmt::img res;

    uint8_t type     = m.type() & CV_MAT_DEPTH_MASK;
    uint8_t channels = m.channels();
  
    size_t width  = m.cols;
    size_t height = m.rows;
    
    if(type == CV_8U){
        if(channels == 1){
            res = abmt::img_gray(width, height, m.data);
        }else if(channels == 3){
            res = abmt::img_rgb(width, height, m.data);
        }if(channels == 4){
            res = abmt::img_rgba(width, height, m.data);
        }
    }else if(type == CV_32F){
        if(channels == 1){
            res = abmt::img_gray32(width, height, m.data);
        }else if(channels == 3){
            res = abmt::img_rgb32(width, height, m.data);
        }if(channels == 4){
            res = abmt::img_rgba32(width, height, m.data);
        }
    }

    return res;
}

cv::Mat opencv::img2mat(abmt::img& img){
    Mat res;
    if(img.type == abmt::image_type::RGB8){
        res = Mat(Size(img.width, img.height), CV_8UC3, img.data.data, Mat::AUTO_STEP);
        //cvtColor(Mat(Size(img.width, img.height), CV_8UC3, img.data.data, Mat::AUTO_STEP), res, cv::COLOR_RGB2BGR);
    }else if(img.type == abmt::image_type::RGBA8){
        cvtColor(Mat(Size(img.width, img.height), CV_8UC4, img.data.data, Mat::AUTO_STEP), res, cv::COLOR_RGBA2BGRA);
    }else if(img.type == abmt::image_type::GRAY8){
        res = Mat(Size(img.width, img.height), CV_8UC1, img.data.data, Mat::AUTO_STEP);
    }else if(img.type == abmt::image_type::RGB32){
        cvtColor(Mat(Size(img.width, img.height), CV_32FC3, img.data.data, Mat::AUTO_STEP), res, cv::COLOR_RGB2BGR);
    }else if(img.type == abmt::image_type::RGBA32){
        cvtColor(Mat(Size(img.width, img.height), CV_32FC4, img.data.data, Mat::AUTO_STEP), res, cv::COLOR_RGBA2BGRA);
    }else if(img.type == abmt::image_type::GRAY32){
        res = Mat(Size(img.width, img.height), CV_32FC1, img.data.data, Mat::AUTO_STEP);
    }else if(img.type == abmt::image_type::JPEG || img.type == abmt::image_type::YUYV){
        abmt::img_rgb converted = img;
        res = Mat(Size(converted.width, converted.height), CV_8UC3, converted.data.data, Mat::AUTO_STEP);
    }else{
        cout << "unable to convert to mat" << endl;
    }
    
    return res;
}

void blur::tick(){
    try{
        Mat gray   = img2mat(in_img);
        out        = in_img.copy_for_dst();
        Mat blured = img2mat(out);
        cv::blur( gray, blured, Size(in_blur_size,in_blur_size) );
    }catch(...){
    	// open cv will print an errormessage
    }
}

void blur_rgb::tick(){
    //auto t = abmt::time::now();
    try{
        Mat gray   = img2mat(in_img);
        out        = in_img.copy_for_dst();
        Mat blured = img2mat(out);
        cv::blur( gray, blured, Size(in_blur_size,in_blur_size) );
    }catch(...){
    	// open cv will print an errormessage
    }
    //cout << "dt: " << t.delta_to_now().us() << endl;
}

void canny::tick(){
    try{
		Mat gray   = img2mat(in_img);
		out        = in_img.copy_for_dst();
		Mat result = img2mat(out);

		Canny( gray, result, in_threshold1, in_threshold2);
    }catch(...){
    	// open cv will print an errormessage
    }
}

void hough_lines::tick(){
    try{
        Mat gray = img2mat(in);
        vector<Vec4i> lines; 
        HoughLinesP(gray, lines, 1, CV_PI/180 , 30, 50, 10 );
        out.clear();
        for( size_t i = 0; i < lines.size(); i++ ){
            Vec4i l = lines[i];
            abmt::vec2 p1 = {(double)l[0], (double)l[1]};
            abmt::vec2 p2 = {(double)l[2], (double)l[3]};
            out.push_back({ p1, p2});
       }
    }catch(...){
    	// open cv will print an errormessage
    }
}

