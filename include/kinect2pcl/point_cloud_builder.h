/*
 * Created by 郭嘉丞 on 15/9/12.
 * Last edited by sugar10w, 2016.2.24
 *
 * 将逐点对应的
 *   depth cv::Mat, CV_32FC1 
 *   registered cv::Mat, CV_8UC3
 * 合成为
 *   PointCloud<PointXYZRGB>
 * 
 */

#ifndef KINECTDATAANALYZER_POINTCLOUDBUILDER_H
#define KINECTDATAANALYZER_POINTCLOUDBUILDER_H

#include <opencv2/opencv.hpp>
#include "common.h"

const int DEPTH_IMAGE_ROWS = 424;
const int DEPTH_IMAGE_COLS = 512;
const int MIN_DEPTH_CM = 60;

class PointCloudBuilder
{
public:
    
    /* 输入数据
     * depthMatrix, 512*424, CV_32FC1
     * imageMatrix, 512*424, CV_8UC3
     */
    PointCloudBuilder(const cv::Mat & depthMatrix, const cv::Mat & imageMatrix);

    // 获取点云
    PointCloudPtr getPointCloud();
    
    // 从 P*[?,?,Z,1]=k[px,py,1] 解出坐标 [X,Y]
    cv::Mat getPointXY(int pixelX, int pixelY, double depth, double p[3][4]);
protected:
    virtual void buildPointCloud();

    cv::Mat depthMat;
    cv::Mat imageMat;
    PointCloudPtr pointCloud;
};


#endif //KINECTDATAANALYZER_POINTCLOUDBUILDER_H
