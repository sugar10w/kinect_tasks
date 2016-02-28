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

#include"load_rgbd.h"

#include<cstdio>
#include<opencv2/opencv.hpp>
#include<pcl/io/pcd_io.h>

#include"kinect2pcl/point_cloud_builder.h"

namespace tinker {
namespace vision {    

using namespace std;


static cv::Mat reload_32f_image(string filename)
{
    ifstream fin(filename.c_str(), ifstream::binary);
    int num_rows, num_cols;
    fin.read((char *) &num_rows, sizeof(int));
    fin.read((char *) &num_cols, sizeof(int));
    cv::Mat mat = cv::Mat::zeros(num_rows, num_cols, CV_32FC1);
    fin.read((char *) mat.data, num_cols * num_rows * 4);
    return mat;
}

PointCloudPtr loadRGBD2Cloud(string depth_file, string reg_file)
{
  cv::Mat depthMat = reload_32f_image(depth_file);
  cv::Mat imageMat = cv::imread(reg_file);
  PointCloudBuilder * builder = new PointCloudBuilder(depthMat, imageMat);
  PointCloudPtr cloud = builder->getPointCloud();
  delete builder;
  return cloud;
}

PointCloudPtr loadRGBD2Cloud(int n)
{
  char buffer[20]; sprintf(buffer, "%d", n); 
  return loadRGBD2Cloud(prefixDepth+buffer+suffixDepth, prefixReg+buffer+suffixReg);
}

cv::Mat loadRGB(int n)
{
	char buffer[20]; sprintf(buffer, "%d", n);
	return cv::imread(prefixRgb+buffer+suffixRgb);
}

cv::Mat loadRegistered(int n)
{
    char buffer[20]; sprintf(buffer, "%d", n);
    return cv::imread(prefixReg+buffer+suffixReg);
}

cv::Mat loadDepth(int n)
{
    char buffer[20]; sprintf(buffer, "%d", n);
    return reload_32f_image(prefixDepth+buffer+suffixDepth);
}

cv::Mat loadDepth(std::string depth_file)
{
    return reload_32f_image(depth_file);
}

}
}
