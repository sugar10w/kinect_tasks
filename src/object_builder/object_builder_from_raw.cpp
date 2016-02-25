/*
 * sugar10w, 2016.1
 * Last edited by sugar10w, 2016.2.25
 *
 * 将物体在场景中的多个不同角度的点云数据拼合为一个三维点云。
 * 
 * 输入数据
 *   深度信息   depth%d.bin       存储512*424个float的二进制文件
 *   校准图片   registered%d.png  存储与depth%d.bin相对应的颜色
 *   原彩色图片 rgb%d.png         (可选)1920*1080图片，提供之则将输出目标所在区域的切图
 *   拍摄信息   kinect_info.txt   提供总帧数、每帧的拍摄时刻信息
 * 照片编号
 *   -1  原始场景
 *   0   放置了旋转展台的场景
 *   >0  旋转中的、不同角度的物品场景
 *
 * 默认情况下，会修正小尺度（5cm左右）物体的2倍深度畸变。
 * 可用 -a 选项关闭此修正。
 *
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include "common.h"
#include "load_rgbd.h"
#include "pcl_filter/point_cloud_minus.h"
#include "object_builder/target_locator.h"
#include "kinect2pcl/image_rebuild.h"

/* 批量输出文件的命名 */
const std::string prefixRes = "res",  // 目标当前帧点云名称
                  prefixCut = "cut";  // 截取图片的名称

int main(int argc, char * argv[])
{
  if (argc==2)
  {
      if (strcmp(argv[1],"-a")==0) ResetZZoom();
  }

  PointCloudPtr cloud_sum(new PointCloud);

  /* 读取-1号和0号点云，相减获取转盘点云  */
  PointCloudPtr cloud_1 = loadRGBD2Cloud(-1);
  PointCloudPtr cloud_0 = loadRGBD2Cloud(0);
  PointCloudPtr rolling_plane = kdTreeMinus(cloud_0, cloud_1);

  //pcl::io::savePCDFile("roll.pcd", *rollingPlane);
  
  /* 确定转盘中心点，截取感兴趣的部位点云，作为其他点云的被减数 */
  PointT center_point = LocateCenterPoint(rolling_plane);
  PointCloudPtr cloud_minus = CutNearCenter( cloud_0, center_point);
 
  std::cout<<"The axis has been located."<<std::endl;
  std::cout<<"["<<center_point.x<<", "<<center_point.y<<", "<<center_point.z<<"]"<<std::endl;

  /* 读取帧信息文件 */
  std::ifstream info_file("kinect_info.txt");
  /* 当前帧拍摄时的时间，以秒记 */
  float time;
  /* 总帧数 */
  int frames;
  info_file >> frames;

  /* 开始处理 */
  char buffer[20]; 
  for (int i=1; i<=frames; ++i)
   {
    /* 将数字i转为字符串buffer */
    sprintf(buffer, "%d", i); 
    
    /* load, cutNearCenter, Minus, 获取目标当前帧物体点云 */
    PointCloudPtr res = kdTreeMinus( CutNearCenter( loadRGBD2Cloud(i) , center_point), cloud_minus);
	
    /* 尝试读取和截取rgb图片 */
	cv::Mat rgb = loadRGB(i);
	if (!rgb.empty() ) cv::imwrite(prefixCut+buffer+".png", GetHDImageFromPointCloud(res, rgb) );

    /* 点云的位置修正，和深度信息修正 */
    MoveToCenter(res, center_point);
    pcl::io::savePCDFile(prefixRes+buffer+".pcd", *res);
    
    /* 旋转之后累加到总点云中 */
    info_file >> time;
    *cloud_sum += *(RotateAfterTime( res , time));

    std::cout<<"processing... "<<i<<"/"<<frames<<std::endl;
  } 

  /* 输出累加得到的点云 */
  pcl::io::savePCDFile("summary.pcd", *cloud_sum);
  

  return 0;

}


