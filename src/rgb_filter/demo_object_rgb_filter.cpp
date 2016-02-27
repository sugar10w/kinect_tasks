/*
 * Created by sugar10w, 2016.2.1
 * Last edited by sugar10w, 2016.2.25
 *
 * 测试 ColorDetector, LineFilter, EntropyFilter, 从BGR图像中分离背景
 *
 */

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "rgb_filter/line_filter.h"
#include "rgb_filter/entropy_filter.h"
#include "rgb_filter/color_fixer.h"
#include "rgb_filter/color_block_filter.h"

using namespace tinker::vision;

/* 获取去除黑色边缘后的图片矩形位置，用于坐标定位和删减无效区域 */
cv::Rect GetMarginRect(cv::Mat & img);
/* 腐蚀膨胀 */
void OpenImage(cv::Mat & img, int range = 3);

int main(int argc, const char * argv[])
{
	/* 获取图片 */
    if (argc!=2)
    {
       std::cout<<"Usage: "<<argv[0]<<" img"<<std::endl;
       return -1;
    }
    std::string filename = argv[1];

    cv::Mat raw_img = cv::imread(filename);

	/* 输入检查 */
    if (raw_img.empty()) 
	{ std::cout<<"Cannot open "<<filename<<std::endl; return -1; }
	
	cv::imshow("raw", raw_img);

    /* 去除图片边缘的黑色部分，并填补图片内部的黑色空洞 */
	cv::Rect raw_margin_rect = GetMarginRect(raw_img);
	cv::Mat img = FixColor( cv::Mat(raw_img, raw_margin_rect), 6);
    
    /* 查找纯色块 */
    ColorBlockFilter color_detector(3);
    cv::Mat color_mask = color_detector.GetMask(img);
	
	//cv::imshow("Fill", color_detector.FillColor(img));

    /* 查找直线（查找时忽略上一次找到的色块） */
    LineFilter line_filter(4, 0.342f);
    line_filter.SetIgnoreMask(color_mask);
    cv::Mat lines_mask = line_filter.GetMask(img);
    
    /* 处理颜色熵 */
    EntropyFilter entropy_filter(5, 0.4);
    cv::Mat entropy_mask = entropy_filter.GetMask(img);


	/* 分别复原到原图的位置 */
	cv::Mat raw_color_mask = cv::Mat::zeros(raw_img.size(), CV_8UC1);
	cv::Mat raw_lines_mask = cv::Mat::zeros(raw_img.size(), CV_8UC1);
	cv::Mat raw_entropy_mask = cv::Mat::zeros(raw_img.size(), CV_8UC1);
	color_mask.copyTo(raw_color_mask(raw_margin_rect));
	lines_mask.copyTo(raw_lines_mask(raw_margin_rect));
	entropy_mask.copyTo(raw_entropy_mask(raw_margin_rect));


	/* 物体 */
	cv::Mat mask = ( raw_entropy_mask & raw_lines_mask ) | raw_color_mask;
    //OpenImage(mask);

	cv::Mat object_img = raw_img.clone();
	object_img.setTo(255, ~mask);
    cv::imshow("object", object_img);
	
	/* 背景 */
	cv::Mat back_mask = ~raw_color_mask & ~raw_entropy_mask & raw_lines_mask;
	OpenImage(back_mask);

	cv::Mat back_img = raw_img.clone();
	back_img.setTo(255,	~back_mask);
	cv::imshow("back", back_img);
	
	/* 直线框架 */
	cv::Mat lines_img = raw_img.clone();
	lines_img.setTo(255, raw_lines_mask);
	cv::imshow("lines", lines_img);
	
	/* 色块区域 */
	cv::Mat color_img = raw_img.clone();
	color_img.setTo(255, ~raw_color_mask);
	cv::imshow("color block", color_img);

	/* 熵区域 */
	cv::Mat entropy_img = raw_img.clone();
	entropy_img.setTo(255, ~raw_entropy_mask);
	cv::imshow("entropy block", entropy_img);

    cv::waitKey(0);
}

/* 获取去除黑色边缘后的图片矩形位置，用于坐标定位和删减无效区域 */
cv::Rect GetMarginRect(cv::Mat & img)
{
    int xmin, xmax, ymin, ymax;
    xmin = xmax = img.cols/2;
    ymin = ymax = img.rows/2;

    if (img.type()==CV_8UC3)
    {
        /* 选几行测试一下 xmin,xmax */
        for (int i=xmin; i>=0; --i) 
			if (img.at<cv::Vec3b>(img.rows/4,i)[0]!=0) xmin = i;
        for (int i=xmin; i>=0; --i)   
		   	if (img.at<cv::Vec3b>(img.rows/2,i)[1]!=0) xmin = i;
        for (int i=xmin; i>=0; --i)    
			if (img.at<cv::Vec3b>(img.rows*3/4,i)[2]!=0) xmin = i;
        
        for (int i=xmax; i<img.cols; ++i) 
			if (img.at<cv::Vec3b>(img.rows/4,i)[0]!=0) xmax = i;
        for (int i=xmax; i<img.cols; ++i)
		   	if (img.at<cv::Vec3b>(img.rows/2,i)[1]!=0) xmax = i;
        for (int i=xmax; i<img.cols; ++i) 
			if (img.at<cv::Vec3b>(img.rows*3/4,i)[2]!=0) xmax = i;

        /* 选几列测试一下 ymin,ymax */
        for (int i=ymin; i>=0; --i) 
			if (img.at<cv::Vec3b>(i,img.cols/4)[0]!=0) ymin = i;
        for (int i=ymin; i>=0; --i) 
			if (img.at<cv::Vec3b>(i,img.cols/2)[1]!=0) ymin = i;
        for (int i=ymin; i>=0; --i) 
			if (img.at<cv::Vec3b>(i,img.cols*3/4)[2]!=0) ymin = i;

        for (int i=ymax; i<img.rows; ++i)
		   	if (img.at<cv::Vec3b>(i,img.cols/4)[0]!=0) ymax = i;
        for (int i=ymax; i<img.rows; ++i) 
			if (img.at<cv::Vec3b>(i,img.cols/2)[1]!=0) ymax = i;
        for (int i=ymax; i<img.rows; ++i)
		   	if (img.at<cv::Vec3b>(i,img.cols*3/4)[2]!=0) ymax = i;
    }
    else
    {
        xmin = ymin = 0;
        xmax = img.cols;
        ymax = img.rows;
    }
	
	/* 返回确定的矩形位置 */
    return cv::Rect(
			xmin,
			ymin, 
			xmax-xmin+1,
			ymax-ymin+1
		);
}

/* 腐蚀膨胀 */
void OpenImage(cv::Mat & img, int range)
{
    if (img.empty()) return;

    //cv::imshow("raw mask", img);

    //cv::Mat resized(img.rows/range, img.cols/range, CV_8UC1);
    //cv::resize(img, resized, resized.size());
    //cv::imshow("resized", resized);

    //static const int erode_kernel_size = 1;
    //cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
    //    cv::Size(2 * erode_kernel_size + 1, 2 * erode_kernel_size + 1),
    //  cv::Point(erode_kernel_size, erode_kernel_size));   
    //cv::erode(resized, resized, erode_kernel);
    //cv::imshow("resized", resized);

    //cv::resize(resized, img, img.size());
    
//------------------------------

    static const int refill_kernel_size = 2;
    static const int erode_kernel_size = 2;
    static const int dilate_kernel_size = 2;

    cv::Mat refill_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * refill_kernel_size + 1, 2 * refill_kernel_size + 1),
        cv::Point(refill_kernel_size, refill_kernel_size));
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * erode_kernel_size + 1, 2 * erode_kernel_size + 1),
        cv::Point(erode_kernel_size, erode_kernel_size));
    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * dilate_kernel_size + 1, 2 * dilate_kernel_size + 1),
        cv::Point(dilate_kernel_size, dilate_kernel_size));

    //cv::imshow("raw mask", img);

    //cv::dilate(img, img, refill_kernel);
    //cv::imshow("1 refill", img);

    cv::erode(img, img, erode_kernel);
    //cv::imshow("2 erode", img);
    
    cv::dilate(img, img, dilate_kernel);
    //cv::imshow("3 dilate", img);
}


