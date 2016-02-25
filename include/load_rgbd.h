/*
 * sugar10w, 2016.1
 *
 * 按照既定的文件名规则，读取点云和图片。
 * 
 * 命名规则
 *   深度图片   "depth%d.bin"
 *   校正图片   "registered%d.png"
 *   原彩色图片 "rgb%d.png"
 *
 */

#ifndef _OBJECTSCAN_LOADRGBD_
#define _OBJECTSCAN_LOADRGBD_

#include<string>
#include<opencv2/opencv.hpp>
#include"common.h"

const std::string prefixDepth = "depth", suffixDepth = ".bin";
const std::string prefixReg = "registered", suffixReg = ".png";
const std::string prefixRgb = "rgb", suffixRgb = ".png";

PointCloudPtr loadRGBD2Cloud(int n);
PointCloudPtr loadRGBD2Cloud(std::string depth_file, std::string reg_file);
cv::Mat loadRGB(int n);

#endif //_OBJECTSCAN_LOADRGBD_
