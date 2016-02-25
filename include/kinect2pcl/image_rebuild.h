/*
 * Created by 郭嘉丞 on 15/9/25.
 * Last edited by sugar10w, 2016.2.21
 *
 * 利用Kinect标定数据，将三维点云重新转换为二维图片。
 * 以及根据三维点云，在高像素(1920*1080)图片中找到对应的位置图片。
 *
 */ 


#ifndef OBJECTFINDER_IMAGEREBUILD_H
#define OBJECTFINDER_IMAGEREBUILD_H

#include <opencv2/opencv.hpp>
#include "common.h"

/* 标记LowRes(512*424)与HiRes(1920*1080)之间关系的一组常数
 * LowRes的上下边缘是无效点，HiRes的左右边缘是无效点 */
extern const int kLowResWidth;
extern const int kLowResHeight, kLowResTop, kLowResBottom;
extern const int kHiResWidth, kHiResLeft, kHiResRight;
extern const int kHiResHeight;
extern const int kLowResActualHeight, kHiResActualWidth;

/* 利用Kinect标定值，将点云中的每个点重新绘制到二维平面上 */
cv::Mat Get2DImageFromPointCloud(PointCloudPtr cloud);

/* 利用Kinect标定值，找到点云在原图片(lowRes或hiRes)中的位置 */
cv::Rect GetHDRectFromPointCloud(PointCloudPtr cloud, bool hiRes = false);

/* 利用Kinect标定值，将点云在图片(lowRes或hiRes)的位置切出来 */
cv::Mat GetHDImageFromPointCloud(PointCloudPtr cloud, cv::Mat & totalImage, bool hiRes = true, bool extend = false);

#endif //OBJECTFINDER_IMAGEREBUILD_H
