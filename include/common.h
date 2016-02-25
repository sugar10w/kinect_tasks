/*
 * Created by 郭嘉丞 on 15/9/12.
 * Last edited by sugar10w, 2016.2.21
 *
 * 为最常用的几个点云库类型提供简写。
 * 定义一些编译宏开关。
 *
 */

#ifndef KINECTDATAANALYZER_COMMON_H
#define KINECTDATAANALYZER_COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* 点 */
typedef pcl::PointXYZRGB PointT;
/* 点云 */
typedef pcl::PointCloud<PointT> PointCloud;
/* 点云指针 */
typedef PointCloud::Ptr PointCloudPtr;

/* DEBUG编译宏开关 */
#define __DEBUG__

#endif //KINECTDATAANALYZER_COMMON_H
