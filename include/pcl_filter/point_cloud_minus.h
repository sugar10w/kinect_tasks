/*
 * Created by sugar10w, 2016.1
 * Last edited by sugar10w, 2016.2.28
 *
 * 利用kdTree进行点云之间的减法运算
 *
 */


#ifndef _OBJECTFINDER_POINTCLOUDMINUS_
#define _OBJECTFINDER_POINTCLOUDMINUS_


#include "common.h"

namespace tinker {
namespace vision {

PointCloudPtr kdTreeMinus(
    PointCloudPtr cloud_a, 
    PointCloudPtr cloud_b,  
    float threshold = 3,
    float distanceThreshold = 3
);

void removeOutlier(
  pcl::PointCloud<PointT>::Ptr cloud,
  pcl::PointCloud<PointT>::Ptr cloud_output,
  int K = 10,
  float distanceThreshold = 1
);

}
}

#endif //_OBJECTFINDER_POINTCLOUDMINUS_

