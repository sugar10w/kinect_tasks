/*
 * Created by sugar10w, 2016.1.19
 * Last edited by sugar10w, 2016.2.25
 *
 * 根据旋转展台的形状，确定转轴和物体位置。
 * 利用物体的位置取出局部图，简化计算量。
 * 针对旋转展台的转速，处理局部点云的旋转。
 *
 * TODO Kinect2对5cm尺度物体，且同平面附近没有其他大型物体干扰时，局部深度信息存在2倍的畸变
 *
 */

#include "object_builder/target_locator.h"

#include<cmath>
#include<vector>
#include<iostream>

#include<pcl/filters/passthrough.h>
#include<pcl/common/transforms.h>
#include<pcl/io/pcd_io.h>

static float Z_ZOOM = RAW_Z_ZOOM;
static float RADIUS_Z = RAW_RADIUS_Z/RAW_Z_ZOOM;

void ResetZZoom(float zoom)
{
    Z_ZOOM = zoom;
    RADIUS_Z = RAW_RADIUS_Z/zoom;
    std::cout<<"Z_ZOOM = "<<Z_ZOOM<<std::endl;
}

// vector<float>排序后，去除最大和最小，中间范围的数字平均值
static float GetVectorCenterAverage(std::vector<float> & vec, float center_ratio = 0.6f)
{
  float sum=0;
  //计算参与计算平均值的范围
  int start=(0.5f-center_ratio/2)*vec.size(), 
      last =(0.5f+center_ratio/2)*vec.size();
  if (start>=last) return vec[vec.size()/2];
  
  //排序并求平均
  std::sort(vec.begin(), vec.end());
  for (int i=start; i<last; ++i) sum+=vec[i];
  return sum/(last-start);
}

// 简单的平移操作
static void MovePointCloud(PointCloudPtr cloud, float x, float y, float z)
{
  int size = cloud->width * cloud->height;
  for (int i=0; i<size; ++i)
  {
    PointT& point = cloud->points[i];
    point.x+=x;
    point.y+=y;
    point.z+=z;
   }
}

// 简单的缩放操作
static void ZoomPointCloud(PointCloudPtr cloud, float x, float y, float z)
{
  int size = cloud->width * cloud->height;
  for (int i=0; i<size; ++i)
  {
    PointT& point = cloud->points[i];
    point.x*=x;
    point.y*=y;
    point.z*=z;
  }
}


// 通过减法得到的点云，求旋转展台的位置，即转轴位置
PointT LocateCenterPoint(PointCloudPtr cloud)
{
  PointT center_point;
  
  // 通过舍弃两侧的点再求平均的方式，求XY坐标(水平和垂直坐标)
  std::vector<float> vX, vY;
  for (int i=0; i<cloud->width*cloud->height; ++i)
  {  
    PointT point = cloud->points[i];
    vX.push_back(point.x);
    vY.push_back(point.y);
  } 
  center_point.x=GetVectorCenterAverage(vX);
  center_point.y=GetVectorCenterAverage(vY);
  
  // 利用获得的XY切割点云
  pcl::PassThrough<PointT> pass;
  PointCloudPtr cloud_1(new PointCloud),
                cloud_2(new PointCloud);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(center_point.x-4.f, center_point.x+4.f);
  pass.filter(*cloud_1);
  
  pass.setInputCloud(cloud_1);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(center_point.y-1.9f, center_point.y+1.9f);
  pass.filter(*cloud_2);
  
  //pcl::io::savePCDFile("roll.pcd", *cloud_2);

  // 得到切割后，点云深度的最小值
  float z_min=10000;
  for (int i=0; i<cloud_2->width*cloud_2->height; ++i)
    if (cloud_2->points[i].z < z_min) z_min = cloud_2->points[i].z;
  
  center_point.z = z_min + RADIUS_Z; //TODO 应该稍后进行此操作

  return center_point;
}


// 剪裁中心点附近的区域
PointCloudPtr CutNearCenter(PointCloudPtr cloud, PointT center)
{
  // 分别剪裁xyz轴
  PointCloudPtr cloud_1(new PointCloud),
                cloud_2(new PointCloud), 
                cloud_3(new PointCloud);
  pcl::PassThrough<PointT> pass;
  
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(center.x-CUT_WIDTH/2, center.x+CUT_WIDTH/2);
  pass.filter(*cloud_1);
  
  pass.setInputCloud(cloud_1);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(center.y, center.y+CUT_HEIGHT+2);
  pass.filter(*cloud_2);
  
  pass.setInputCloud(cloud_2);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(center.z-CUT_WIDTH/2, center.z+EXTRA_DEPTH);
  pass.filter(*cloud_3);
  
  return cloud_3;
}

// 移到原点(将旋转轴与Y轴对齐)
PointCloudPtr MoveToCenter(PointCloudPtr cloud , PointT center)
{
   MovePointCloud(cloud, -center.x, -center.y-2, -center.z);
   ZoomPointCloud(cloud, 1, 1, Z_ZOOM);
   return cloud;
}

PointCloudPtr RotateAfterTime(PointCloudPtr cloud, float time)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  PointCloudPtr cloud_output(new PointCloud);
  float th=ROTATING_SPEED*time;

  transform(0,0)=cos(th);
  transform(0,2)=-sin(th);
  transform(2,0)=sin(th);
  transform(2,2)=cos(th);
  pcl::transformPointCloud(*cloud, *cloud_output, transform);
  
  return cloud_output;
}

