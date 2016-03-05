/*
 * Created by sugar10w, 2016.3.2
 * Last edited by sugar10w, 2016.3.2
 *
 * 使用PCLVisualizer实时显示三维点云
 * 并标记其中的找到的物体 
 * 
 * 使用libfreenect2连接Kinect2
 * 
 */

#include <signal.h>
#include <cstdio>
#include <cstdlib>

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <kinect2pcl/point_cloud_builder.h>
#include "object_builder/cluster_divider.h"
#include "kinect2pcl/point_cloud_builder.h"
#include "rgb_filter/rgb_object_filter.h"
#include "pcl_filter/plane_filter.h"


using namespace tinker::vision;
using namespace libfreenect2;


// 显示点云的窗口 PCD Viewer
pcl::visualization::PCLVisualizer viewer("PCD Viewer");

// 全局控制开关
bool protonect_shutdown = false; 

// 处理SIGNIT(InterruptKey)信号，停止显示
void sigint_handler(int signal_number)
{
  protonect_shutdown = true;
}

// 从蒙版和源数据获取点云
PointCloudPtr GetCloudFromMask(cv::Mat& raw_img, cv::Mat& depth_img, cv::Mat mask)
{
    cv::Mat depth = depth_img.clone();
    depth.setTo(0, ~mask);
    PointCloudBuilder builder(depth, raw_img);
    PointCloudPtr cloud = builder.getPointCloud();
    return cloud;
} 

int main(int argc, char *argv[])
{


  // --------------------------------------------------------------------------
  // 配置 Kinect2

  // 创建一个Debug级别的Logger
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  // 初始化对象
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  std::string serial;
  // 查找Kinect2设备
  if(freenect2.enumerateDevices() == 0)
  { std::cout << "no device connected!" << std::endl; return -1; }
  serial = freenect2.getDefaultDeviceSerialNumber();
  // 打开Kinect2设备
  dev = freenect2.openDevice(serial);
  if(dev == 0)
  { std::cout << "failure opening device!" << std::endl; return -1; }

  // 改绑SIGINT(InterruptKey)信号，用于安全退出
  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  // 配置Kinect2的监听对象，用于获取总数据帧
  //   Color(1920*1080 32-bit BGRX)
  //   Ir(512*424 float)
  //   Depth(512*424 float mm)
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  // 开始运行
  dev->start();
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  // 配置校准器registration (使用默认参数)
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  // --------------------------------------------------------------------------
  // 开始循环
  while(!protonect_shutdown && !viewer.wasStopped())
  {

    // 等待接收总数据帧
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // 校准
    registration->apply(rgb, depth, &undistorted, &registered);

    // 选取depth
    cv::Mat depthMatrix(depth->height, depth->width, CV_32FC1, depth->data);
    // 选取rgb
    cv::Mat rgbMatrix(rgb->height, rgb->width , CV_8UC4, rgb->data);
    // 选取registered
    cv::Mat registeredMatrix(registered.height, registered.width, CV_8UC4, registered.data);


    // 预处理 depth, rgb, registered
    depthMatrix /= 4500.0f;
    cv::cvtColor(rgbMatrix, rgbMatrix, CV_BGRA2BGR);
    cv::cvtColor(registeredMatrix, registeredMatrix, CV_BGRA2BGR);
    // 获取点云cloud
    PointCloudBuilder builder(depthMatrix, registeredMatrix);
    PointCloudPtr cloud_raw = builder.getPointCloud();

    // RGB蒙版
    RgbObjectFilter rgb_filter(rgbMatrix);
    cv::Mat object_mask = rgb_filter.GetObjectMask();
    cv::Mat back_mask   = rgb_filter.GetBackMask();
    // 获取背景图层，分离得到墙面模型
    PointCloudPtr cloud_back = GetCloudFromMask(rgbMatrix, depthMatrix, back_mask);
    PlaneFilter plane_filter(cloud_back);
    // 获取物品图层，并做简单处理
    PointCloudPtr cloud_object = GetCloudFromMask(rgbMatrix, depthMatrix, object_mask);
    plane_filter.Filter(cloud_object);

    // 计算用于展示的点云
    PointCloudPtr cloud_display(new PointCloud);
    float leaf_size_ = 2.0f;
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_raw);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud_display);
    *cloud_display += *cloud_object;    


    // 显示点云
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(cloud_display);
    viewer.addPointCloud<PointT> (cloud_display, rgb_handler, "input_cloud");
    
    // 绘制目标的位置
    ClusterDivider cluster_divider(cloud_object);
    std::vector<ObjectCluster> clusters = cluster_divider.GetDividedCluster();
    for (int i=0; i<clusters.size(); ++i)
    {
        clusters[i].DrawBoundingBox(viewer, i);
    }

    // 刷新viewer
    viewer.spinOnce();
    
    // 释放总数据帧
    listener.release(frames);
  }

  // 通过InterruptKey或者关闭viewer退出循环后
  // 关闭设备
  dev->stop();
  dev->close();

  delete registration;

  return 0;
}
